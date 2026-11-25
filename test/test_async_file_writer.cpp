// Tests for AsyncFileWriter. A FIFO whose reader stops reading stands in for a
// stalled disk: it blocks write(2) in the kernel the same way.

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "microstrain_inertial_driver_common/utils/async_file_writer.h"

using microstrain::AsyncFileWriter;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

static std::vector<uint8_t> makeData(const size_t length, const uint8_t seed) {
  std::vector<uint8_t> data(length);
  for (size_t i = 0; i < length; i++)
    data[i] = static_cast<uint8_t>(seed + i * 31);
  return data;
}

static std::vector<uint8_t> readFile(const std::string &path) {
  std::ifstream file(path, std::ios::binary);
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(file),
                              std::istreambuf_iterator<char>());
}

// Reads a FIFO to the end, stopping for stall_length after stall_after has
// passed
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

static void readFifo(const int fd, const milliseconds stall_after, const milliseconds stall_length,
                     std::vector<uint8_t>* received)
{
  const Clock::time_point start = Clock::now();
  bool stalled = false;
  uint8_t buffer[4096];
  while (true) {
    if (!stalled && Clock::now() - start >= stall_after) {
      std::this_thread::sleep_for(stall_length);
      stalled = true;
    }
    const ssize_t count = read(fd, buffer, sizeof(buffer));
    if (count <= 0)
      break;
    received->insert(received->end(), buffer, buffer + count);
  }
  close(fd);
}

class AsyncFileWriterTest : public ::testing::Test {
protected:
  void SetUp() override {
    char directory_template[] = "/tmp/async_file_writer_test_XXXXXX";
    ASSERT_NE(mkdtemp(directory_template), nullptr);
    directory_ = directory_template;
  }

  void TearDown() override {
    for (const std::string &name : {"file.bin", "fifo.bin"})
      unlink((directory_ + "/" + name).c_str());
    rmdir(directory_.c_str());
  }

  std::string directory_;
};

TEST_F(AsyncFileWriterTest, WritesEveryByteInOrder) {
  const std::string path = directory_ + "/file.bin";
  std::vector<uint8_t> expected;
  {
    AsyncFileWriter writer;
    ASSERT_TRUE(writer.open(path));
    for (uint8_t i = 0; i < 200; i++) {
      const std::vector<uint8_t> chunk = makeData(37 + i * 13, i);
      ASSERT_TRUE(writer.write(chunk.data(), chunk.size()));
      expected.insert(expected.end(), chunk.begin(), chunk.end());
    }
    writer.close();
    ASSERT_TRUE(writer.waitUntilIdle(milliseconds(5000)));
    EXPECT_EQ(readFile(path), expected);
  }
  EXPECT_EQ(readFile(path), expected);
}

TEST_F(AsyncFileWriterTest, DestructorWritesQueuedData) {
  const std::string path = directory_ + "/file.bin";
  const std::vector<uint8_t> data = makeData(100000, 7);
  {
    AsyncFileWriter writer;
    ASSERT_TRUE(writer.open(path));
    ASSERT_TRUE(writer.write(data.data(), data.size()));
  }
  EXPECT_EQ(readFile(path), data);
}

TEST_F(AsyncFileWriterTest, IgnoresWritesWhenClosed) {
  const std::string path = directory_ + "/file.bin";
  AsyncFileWriter writer;
  const std::vector<uint8_t> data = makeData(10, 1);
  EXPECT_FALSE(writer.isOpen());
  EXPECT_TRUE(writer.write(data.data(), data.size()));
  ASSERT_TRUE(writer.open(path));
  EXPECT_TRUE(writer.isOpen());
  writer.close();
  EXPECT_FALSE(writer.isOpen());
  EXPECT_TRUE(writer.write(data.data(), data.size()));
  ASSERT_TRUE(writer.waitUntilIdle(milliseconds(5000)));
  EXPECT_TRUE(readFile(path).empty());
  EXPECT_EQ(writer.droppedBytes(), 0u);
}

TEST_F(AsyncFileWriterTest, OpenFailureIsReported) {
  AsyncFileWriter writer;
  EXPECT_FALSE(writer.open(directory_ + "/does/not/exist.bin"));
  EXPECT_FALSE(writer.isOpen());
}

TEST_F(AsyncFileWriterTest, ReopenWithoutTruncateAppends) {
  const std::string path = directory_ + "/file.bin";
  const std::vector<uint8_t> first = makeData(1000, 1);
  const std::vector<uint8_t> second = makeData(500, 2);
  AsyncFileWriter writer;
  ASSERT_TRUE(writer.open(path));
  writer.write(first.data(), first.size());
  writer.close();
  ASSERT_TRUE(writer.open(path, false));
  writer.write(second.data(), second.size());
  writer.close();
  ASSERT_TRUE(writer.waitUntilIdle(milliseconds(5000)));

  std::vector<uint8_t> expected = first;
  expected.insert(expected.end(), second.begin(), second.end());
  EXPECT_EQ(readFile(path), expected);
}

TEST_F(AsyncFileWriterTest, StalledDiskNeverBlocksWriter) {
  const std::string path = directory_ + "/fifo.bin";
  ASSERT_EQ(mkfifo(path.c_str(), 0600), 0);
  std::vector<uint8_t> received;
  const FifoEnds fifo = openFifo(path);
  ASSERT_GE(fifo.read_fd, 0);
  std::thread reader(readFifo, fifo.read_fd, milliseconds(500), milliseconds(1500), &received);

  AsyncFileWriter writer;
  ASSERT_TRUE(writer.open(path));
  close(fifo.placeholder_write_fd);

  // 512 byte reads at ~73 KB/s for 3 s, as the driver does with support data
  // enabled
  std::vector<uint8_t> sent;
  Clock::duration longest_write(0);
  const Clock::time_point start = Clock::now();
  uint8_t seed = 0;
  while (Clock::now() - start < milliseconds(3000)) {
    const std::vector<uint8_t> chunk = makeData(512, seed++);
    const Clock::time_point before = Clock::now();
    EXPECT_TRUE(writer.write(chunk.data(), chunk.size()));
    longest_write = std::max(longest_write, Clock::now() - before);
    sent.insert(sent.end(), chunk.begin(), chunk.end());
    std::this_thread::sleep_for(milliseconds(7));
  }
  writer.close();
  reader.join();

  EXPECT_LT(longest_write, milliseconds(50));
  EXPECT_EQ(writer.droppedBytes(), 0u);
  EXPECT_TRUE(received == sent);
}

TEST_F(AsyncFileWriterTest, DropsInsteadOfBlockingWhenQueueIsFull) {
  const std::string path = directory_ + "/fifo.bin";
  ASSERT_EQ(mkfifo(path.c_str(), 0600), 0);
  std::vector<uint8_t> received;
  const FifoEnds fifo = openFifo(path);
  ASSERT_GE(fifo.read_fd, 0);
  std::thread reader(readFifo, fifo.read_fd, milliseconds(0), milliseconds(1000), &received);

  const size_t max_queued = 64 * 1024;
  AsyncFileWriter writer(max_queued);
  ASSERT_TRUE(writer.open(path));
  close(fifo.placeholder_write_fd);

  size_t total = 0;
  const Clock::time_point start = Clock::now();
  for (uint8_t i = 0; i < 100; i++) {
    const std::vector<uint8_t> chunk = makeData(4096, i);
    writer.write(chunk.data(), chunk.size());
    total += chunk.size();
  }
  const Clock::duration elapsed = Clock::now() - start;
  writer.close();
  reader.join();

  EXPECT_LT(elapsed, milliseconds(250));
  EXPECT_GT(writer.droppedBytes(), 0u);
  EXPECT_EQ(received.size() + writer.droppedBytes(), total);
}

TEST_F(AsyncFileWriterTest, ReopeningSamePathDoesNotMixOldAndNewData)
{
  // Restarting a recording at the same path: data still queued for the old file must not end up in, or leave a hole
  // in, the new one
  const std::string path = directory_ + "/file.bin";
  const std::vector<uint8_t> old_data = makeData(4 * 1024 * 1024, 3);
  const std::vector<uint8_t> new_data = makeData(1000, 4);
  AsyncFileWriter writer;
  ASSERT_TRUE(writer.open(path));
  for (size_t offset = 0; offset < old_data.size(); offset += 4096)
    writer.write(old_data.data() + offset, 4096);
  writer.close();
  ASSERT_TRUE(writer.open(path));
  writer.write(new_data.data(), new_data.size());
  writer.close();
  ASSERT_TRUE(writer.waitUntilIdle(milliseconds(10000)));
  EXPECT_TRUE(readFile(path) == new_data);
}

TEST_F(AsyncFileWriterTest, DestructorDoesNotHangOnStalledDisk)
{
  const std::string path = directory_ + "/fifo.bin";
  ASSERT_EQ(mkfifo(path.c_str(), 0600), 0);
  std::vector<uint8_t> received;
  const FifoEnds fifo = openFifo(path);
  ASSERT_GE(fifo.read_fd, 0);
  std::thread reader(readFifo, fifo.read_fd, milliseconds(0), milliseconds(3000), &received);

  const std::vector<uint8_t> data = makeData(256 * 1024, 5);
  auto writer = std::make_unique<AsyncFileWriter>(AsyncFileWriter::DEFAULT_MAX_QUEUED_BYTES, milliseconds(200));
  ASSERT_TRUE(writer->open(path));
  close(fifo.placeholder_write_fd);
  writer->write(data.data(), data.size());

  const Clock::time_point before = Clock::now();
  writer.reset();
  const Clock::duration destroy_time = Clock::now() - before;
  EXPECT_LT(destroy_time, milliseconds(1000));

  // The detached writer thread still finishes writing once the disk recovers
  reader.join();
  EXPECT_TRUE(received == data);
}
