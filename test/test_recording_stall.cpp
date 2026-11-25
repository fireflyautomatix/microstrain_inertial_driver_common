// Reproduces, without hardware, the GpsDataInterrupted failure seen while
// recording a raw binary file.
//
// A pseudo terminal stands in for the GQ7 serial port and a feeder thread
// streams valid MIP packets into it at the rate observed in the field (~73
// KB/s). The raw file is a FIFO whose reader stops reading for 1.5 s, which
// blocks write(2) exactly like a stalled disk does. A loop that mimics the
// driver's parsing timer reads the port through RosConnection + mip::Interface
// and records when every packet is dispatched.
//
// In the field, the stall blocked the whole driver thread for 1-1.6 s and the
// bin file still had every byte. These tests require that packets keep being
// dispatched on time while the file is stalled, and that the raw file still
// receives every byte.

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "mip/mip_interface.hpp"

#include "microstrain_inertial_driver_common/utils/mip/ros_connection.h"

using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

static constexpr size_t BYTES_PER_SECOND =
    73000; // Average raw file rate on amp_101 with support data
static constexpr size_t PACKET_PAYLOAD =
    100; // Bytes of payload per synthetic packet
static constexpr uint8_t DESCRIPTOR_SET = 0x80; // Sensor data set
static constexpr auto TIMER_PERIOD =
    milliseconds(5); // 200 Hz, the driver's spin rate on amp_101
static constexpr auto STALL_START =
    milliseconds(1000); // When the raw file reader stops reading
static constexpr auto STALL_LENGTH = milliseconds(1500); // How long it stops
static constexpr auto FEED_LENGTH =
    milliseconds(4000); // How long the device streams data
// Largest acceptable gap between dispatched packets. Well under the 1.5 s stall, with room for a loaded CI machine
static constexpr auto MAX_ALLOWED_GAP = milliseconds(250);

// Builds a MIP packet: one GPS timestamp field and one padding field whose
// first 4 bytes carry a sequence number
static std::vector<uint8_t> makePacket(const uint32_t sequence) {
  std::vector<uint8_t> packet;
  packet.reserve(4 + PACKET_PAYLOAD + 2);
  packet.push_back(0x75);
  packet.push_back(0x65);
  packet.push_back(DESCRIPTOR_SET);
  packet.push_back(static_cast<uint8_t>(PACKET_PAYLOAD));

  // Shared GPS timestamp field (0xD3): tow (double), week (u16), valid flags
  // (u16), big endian
  const uint8_t gps_field[14] = {14, 0xD3, 0, 0,    0,    0,    0,
                                 0,  0,    0, 0x08, 0xC0, 0x00, 0x03};
  packet.insert(packet.end(), std::begin(gps_field), std::end(gps_field));

  // Padding field with an unused descriptor, carrying the sequence number
  const uint8_t pad_length =
      static_cast<uint8_t>(PACKET_PAYLOAD - sizeof(gps_field));
  packet.push_back(pad_length);
  packet.push_back(0xFE);
  packet.push_back(static_cast<uint8_t>(sequence >> 24));
  packet.push_back(static_cast<uint8_t>(sequence >> 16));
  packet.push_back(static_cast<uint8_t>(sequence >> 8));
  packet.push_back(static_cast<uint8_t>(sequence));
  packet.resize(packet.size() + pad_length - 6, 0xAA);

  // Fletcher checksum over header and payload
  uint8_t a = 0, b = 0;
  for (const uint8_t byte : packet) {
    a = static_cast<uint8_t>(a + byte);
    b = static_cast<uint8_t>(b + a);
  }
  packet.push_back(a);
  packet.push_back(b);
  return packet;
}

// Collects what the parser dispatched, from the packet callback
struct Dispatched {
  std::vector<uint32_t> sequences;
  std::vector<Clock::time_point> times;
};

static void handlePacket(void *context, const mip::C::mip_packet_view *packet,
                         mip::C::mip_timestamp /* timestamp */) {
  auto *dispatched = static_cast<Dispatched *>(context);
  const uint8_t *payload = mip::C::mip_packet_payload(packet);
  const size_t pad_offset = 14 + 2;
  const uint32_t sequence =
      (static_cast<uint32_t>(payload[pad_offset]) << 24) |
      (static_cast<uint32_t>(payload[pad_offset + 1]) << 16) |
      (static_cast<uint32_t>(payload[pad_offset + 2]) << 8) |
      static_cast<uint32_t>(payload[pad_offset + 3]);
  dispatched->sequences.push_back(sequence);
  dispatched->times.push_back(Clock::now());
}

// Reads the FIFO the raw file is written to, stopping for STALL_LENGTH once
// STALL_START has passed
// Opens the read end of a FIFO without waiting for a writer, so a failure is reported instead of the writer's open
// hanging. A temporary write end is returned too, so the reader does not see end of file before the real writer opens.
// Close it once the real writer has the FIFO open
struct FifoEnds
{
  int read_fd = -1;
  int placeholder_write_fd = -1;
};

static FifoEnds openFifo(const std::string& path)
{
  FifoEnds ends;
  ends.read_fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
  if (ends.read_fd < 0)
    return ends;
  ends.placeholder_write_fd = open(path.c_str(), O_WRONLY | O_NONBLOCK);
  fcntl(ends.read_fd, F_SETFL, fcntl(ends.read_fd, F_GETFL) & ~O_NONBLOCK);

  // Make the pipe as small as possible, so a stalled reader blocks the writer almost immediately
  fcntl(ends.read_fd, F_SETPIPE_SZ, 4096);
  return ends;
}

static void readFifo(const int fd, const Clock::time_point start, std::vector<uint8_t>* received)
{

  bool stalled = false;
  uint8_t buffer[4096];
  while (true) {
    if (!stalled && Clock::now() - start >= STALL_START) {
      std::this_thread::sleep_for(STALL_LENGTH);
      stalled = true;
    }
    const ssize_t count = read(fd, buffer, sizeof(buffer));
    if (count <= 0)
      break; // Writer closed the file
    received->insert(received->end(), buffer, buffer + count);
  }
  close(fd);
}

class RecordingStallTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("recording_stall_test");

    // Pseudo terminal that stands in for the device's serial port
    char slave_name[256] = {0};
    ASSERT_EQ(openpty(&master_fd_, &slave_fd_, slave_name, nullptr, nullptr),
              0);
    struct termios settings;
    tcgetattr(slave_fd_, &settings);
    cfmakeraw(&settings);
    tcsetattr(slave_fd_, TCSANOW, &settings);
    port_ = slave_name;

    // FIFO that stands in for the raw file on a stalled disk
    char directory_template[] = "/tmp/recording_stall_test_XXXXXX";
    ASSERT_NE(mkdtemp(directory_template), nullptr);
    directory_ = directory_template;
    fifo_path_ = directory_ + "/raw.bin";
    ASSERT_EQ(mkfifo(fifo_path_.c_str(), 0600), 0);
  }

  void TearDown() override {
    close(master_fd_);
    close(slave_fd_);
    unlink(fifo_path_.c_str());
    rmdir(directory_.c_str());
  }

  std::shared_ptr<rclcpp::Node> node_;
  int master_fd_ = -1;
  int slave_fd_ = -1;
  std::string port_;
  std::string directory_;
  std::string fifo_path_;
};

TEST_F(RecordingStallTest, StalledRawFileDoesNotStallParsing) {
  auto connection = std::make_shared<microstrain::RosConnection>(node_.get());
  ASSERT_TRUE(connection->connect(node_.get(), port_, 921600));
  mip::Interface device(connection.get(), connection->parseTimeout(),
                        connection->baseReplyTimeout());

  Dispatched dispatched;
  mip::C::mip_dispatch_handler handler;
  device.registerPacketCallback(handler, DESCRIPTOR_SET, false, &handlePacket,
                                &dispatched);

  // Start recording. Opening a FIFO for writing waits until the reader has it
  // open
  const Clock::time_point start = Clock::now();
  std::vector<uint8_t> recorded;
  const FifoEnds fifo = openFifo(fifo_path_);
  ASSERT_GE(fifo.read_fd, 0);
  std::thread reader(readFifo, fifo.read_fd, start, &recorded);
  ASSERT_TRUE(connection->updateRecordingState(true, fifo_path_));
  close(fifo.placeholder_write_fd);

  // Stream packets into the port at the field data rate
  std::vector<uint8_t> sent;
  std::mutex sent_mutex;
  std::atomic<uint32_t> packets_sent{0};
  std::thread feeder([&]() {
    const size_t packet_size = makePacket(0).size();
    const auto period = milliseconds(10);
    const size_t packets_per_period =
        (BYTES_PER_SECOND / 100) / packet_size + 1;
    uint32_t sequence = 0;
    auto next = Clock::now();
    while (Clock::now() - start < FEED_LENGTH) {
      std::vector<uint8_t> chunk;
      for (size_t i = 0; i < packets_per_period; i++) {
        const std::vector<uint8_t> packet = makePacket(sequence++);
        chunk.insert(chunk.end(), packet.begin(), packet.end());
      }
      {
        std::lock_guard<std::mutex> lock(sent_mutex);
        sent.insert(sent.end(), chunk.begin(), chunk.end());
      }
      size_t written = 0;
      while (written < chunk.size()) {
        const ssize_t count =
            write(master_fd_, chunk.data() + written, chunk.size() - written);
        if (count > 0)
          written += static_cast<size_t>(count);
      }
      packets_sent = sequence;
      next += period;
      std::this_thread::sleep_until(next);
    }
  });

  // Mimic the driver's parsing timer until every packet has been dispatched, or
  // give up
  const auto deadline = start + FEED_LENGTH + STALL_LENGTH + milliseconds(3000);
  auto next_tick = Clock::now();
  while (Clock::now() < deadline) {
    // Same as NodeCommon::parseAndPublishMain
    device.update(10);
    microstrain::drainDevice(&device, *connection, milliseconds(5));
    if (Clock::now() - start > FEED_LENGTH &&
        dispatched.sequences.size() >= packets_sent)
      break;
    next_tick += TIMER_PERIOD;
    std::this_thread::sleep_until(next_tick);
  }
  feeder.join();

  // Stop recording and wait for the reader to see the end of the file
  connection->updateRecordingState(false, fifo_path_);
  reader.join();

  // Every packet was dispatched, in order
  ASSERT_EQ(dispatched.sequences.size(), packets_sent.load());
  for (size_t i = 0; i < dispatched.sequences.size(); i++)
    ASSERT_EQ(dispatched.sequences[i], i)
        << "packet " << i << " lost or out of order";

  // No gap in dispatching while data was streaming. In the field this gap is
  // what tripped GpsDataInterrupted
  Clock::duration max_gap(0);
  for (size_t i = 1; i < dispatched.times.size(); i++)
    if (dispatched.times[i] - start < FEED_LENGTH)
      max_gap =
          std::max(max_gap, dispatched.times[i] - dispatched.times[i - 1]);
  EXPECT_LT(max_gap, MAX_ALLOWED_GAP)
      << "parsing stalled for "
      << std::chrono::duration_cast<milliseconds>(max_gap).count()
      << " ms while the raw file was stalled";

  // The raw file got every byte, exactly as it came off the port
  std::lock_guard<std::mutex> lock(sent_mutex);
  EXPECT_EQ(recorded.size(), sent.size());
  EXPECT_TRUE(recorded == sent)
      << "raw file does not match the bytes sent by the device";
}

TEST_F(RecordingStallTest, DrainClearsBacklogInOneCall)
{
  auto connection = std::make_shared<microstrain::RosConnection>(node_.get());
  ASSERT_TRUE(connection->connect(node_.get(), port_, 921600));
  mip::Interface device(connection.get(), connection->parseTimeout(), connection->baseReplyTimeout());

  Dispatched dispatched;
  mip::C::mip_dispatch_handler handler;
  device.registerPacketCallback(handler, DESCRIPTOR_SET, false, &handlePacket, &dispatched);

  // Queue up a backlog, as if the driver had been blocked, until the pseudo terminal will not take any more
  fcntl(master_fd_, F_SETFL, fcntl(master_fd_, F_GETFL) | O_NONBLOCK);
  size_t queued_bytes = 0;
  uint32_t queued_packets = 0;
  while (queued_bytes < 200 * 1024)
  {
    const std::vector<uint8_t> packet = makePacket(queued_packets);
    const ssize_t count = write(master_fd_, packet.data(), packet.size());
    if (count > 0)
      queued_bytes += static_cast<size_t>(count);  // A partial packet is still read, it just is not dispatched
    if (count != static_cast<ssize_t>(packet.size()))
      break;
    queued_packets++;
  }
  ASSERT_GT(queued_bytes, 4096u) << "pseudo terminal accepted too little data to test with";

  // One regular update reads a single chunk, which is what made a backlog slow to clear
  ASSERT_TRUE(device.update(10));
  const size_t first_read = connection->lastRecvLength();
  EXPECT_LE(first_read, 512u);

  // Draining reads everything that is waiting in one call
  const size_t drained = microstrain::drainDevice(&device, *connection, milliseconds(500));
  EXPECT_EQ(first_read + drained, queued_bytes);
  EXPECT_EQ(dispatched.sequences.size(), queued_packets);

  // With nothing waiting, draining returns straight away
  const Clock::time_point before = Clock::now();
  EXPECT_EQ(microstrain::drainDevice(&device, *connection, milliseconds(500)), 0u);
  EXPECT_LT(Clock::now() - before, milliseconds(50));
}
