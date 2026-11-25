/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ASYNC_FILE_WRITER_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ASYNC_FILE_WRITER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace microstrain {

/**
 * Writes a binary file from a background thread, so the thread producing the
 * data never waits on the disk. Data is queued in memory up to a limit. If the
 * disk falls further behind than that, new data is dropped and counted instead
 * of blocking the producer.
 */
class AsyncFileWriter {
public:
  static constexpr size_t DEFAULT_MAX_QUEUED_BYTES = 8 * 1024 * 1024;
  static constexpr std::chrono::milliseconds DEFAULT_DESTROY_TIMEOUT{2000};

  /**
   * \brief Starts the writer thread
   * \param max_queued_bytes Most data that may wait in memory for the disk
   * before new data is dropped \param destroy_timeout Longest the destructor
   * waits for queued data to be written. After that, the writer thread finishes
   * on its own, so a stalled disk cannot hang the caller
   */
  explicit AsyncFileWriter(
      size_t max_queued_bytes = DEFAULT_MAX_QUEUED_BYTES,
      std::chrono::milliseconds destroy_timeout = DEFAULT_DESTROY_TIMEOUT);

  /**
   * \brief Writes out everything still queued and closes the file, waiting at
   * most destroy_timeout
   */
  ~AsyncFileWriter();

  AsyncFileWriter(const AsyncFileWriter &) = delete;
  AsyncFileWriter &operator=(const AsyncFileWriter &) = delete;

  /**
   * \brief Opens a new file. Any open file is closed once its queued data has
   * been written \param path Path of the file to open \param truncate Whether
   * to discard existing contents of the file, or append to them. Truncation is
   * done by the writer thread after any previous file has been closed, so
   * reopening the same path is safe \return Whether the file could be opened
   */
  bool open(const std::string &path, bool truncate = true);

  /**
   * \brief Closes the file once its queued data has been written. Does not wait
   * for that to happen
   */
  void close();

  /**
   * \brief Whether a file is open for writing
   */
  bool isOpen() const;

  /**
   * \brief Queues data to be written to the open file. Never waits on the disk
   * \param data Data to write
   * \param length Number of bytes to write
   * \return False if the data was dropped because the queue was full. True if
   * it was queued, or no file is open
   */
  bool write(const uint8_t *data, size_t length);

  /**
   * \brief Waits until everything queued so far has been written and any
   * pending close has finished \param timeout Longest time to wait \return
   * Whether the writer became idle before the timeout
   */
  bool waitUntilIdle(std::chrono::milliseconds timeout);

  /**
   * \brief Total number of bytes that did not make it to the file, because the
   * queue was full or the write failed
   */
  uint64_t droppedBytes() const;

private:
  // One unit of work for the writer thread: either data to write, or a switch
  // to a new file (or to no file)
  struct Item {
    std::vector<uint8_t> data;
    bool switch_file = false;
    std::unique_ptr<std::ofstream> file;
    std::string truncate_path; /// If not empty, truncate this file after
                               /// switching to it
  };

  // Everything the writer thread uses. Shared, so the thread can outlive this
  // object if the destructor stops waiting
  struct State {
    explicit State(size_t max_queued_bytes)
        : max_queued_bytes(max_queued_bytes) {}

    const size_t max_queued_bytes;
    std::mutex mutex;
    std::condition_variable work_cv; /// Signals the writer thread that there is
                                     /// work or it should stop
    std::condition_variable idle_cv; /// Signals waiters that the queue is empty
    std::deque<Item> queue;
    size_t queued_bytes = 0;
    bool open = false; /// Whether the most recent open or close request left a
                       /// file open
    bool busy = false; /// Whether the writer thread is working on an item it
                       /// removed from the queue
    bool stop = false;
    bool finished = false; /// Set by the writer thread when it has closed the
                           /// file and is about to exit
    std::atomic<uint64_t> dropped_bytes{0};
  };

  static void run(std::shared_ptr<State> state);

  std::shared_ptr<State> state_;
  std::chrono::milliseconds destroy_timeout_;
  std::thread thread_;
};

} // namespace microstrain

#endif // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_ASYNC_FILE_WRITER_H
