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

#include <unistd.h>

#include <utility>

#include "microstrain_inertial_driver_common/utils/async_file_writer.h"

namespace microstrain {

constexpr size_t AsyncFileWriter::DEFAULT_MAX_QUEUED_BYTES;
constexpr std::chrono::milliseconds AsyncFileWriter::DEFAULT_DESTROY_TIMEOUT;

// Small reads from the device are merged into chunks up to this size, to avoid
// one queue entry per read
static constexpr size_t MAX_CHUNK_BYTES = 64 * 1024;

AsyncFileWriter::AsyncFileWriter(
    const size_t max_queued_bytes,
    const std::chrono::milliseconds destroy_timeout)
    : state_(std::make_shared<State>(max_queued_bytes)),
      destroy_timeout_(destroy_timeout) {
  thread_ = std::thread(&AsyncFileWriter::run, state_);
}

AsyncFileWriter::~AsyncFileWriter() {
  std::unique_lock<std::mutex> lock(state_->mutex);
  state_->stop = true;
  state_->work_cv.notify_one();

  // Give the writer thread a chance to write out what is queued, but do not
  // hang the caller on a stalled disk. If it is still busy, it keeps its own
  // reference to the state and finishes on its own
  const bool finished = state_->idle_cv.wait_for(
      lock, destroy_timeout_, [this]() { return state_->finished; });
  lock.unlock();
  if (finished)
    thread_.join();
  else
    thread_.detach();
}

bool AsyncFileWriter::open(const std::string &path, const bool truncate) {
  // Open on the caller's thread so failures can be reported to it. The file is
  // opened in append mode so that it is not truncated here: data for a previous
  // file at the same path may still be waiting to be written
  auto file = std::make_unique<std::ofstream>(
      path, std::ios::out | std::ios::binary | std::ios::app);
  if (!file->is_open())
    return false;

  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    Item item;
    item.switch_file = true;
    item.file = std::move(file);
    if (truncate)
      item.truncate_path = path;
    state_->queue.push_back(std::move(item));
    state_->open = true;
  }
  state_->work_cv.notify_one();
  return true;
}

void AsyncFileWriter::close() {
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!state_->open)
      return;
    Item item;
    item.switch_file = true;
    state_->queue.push_back(std::move(item));
    state_->open = false;
  }
  state_->work_cv.notify_one();
}

bool AsyncFileWriter::isOpen() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->open;
}

bool AsyncFileWriter::write(const uint8_t *data, const size_t length) {
  if (length == 0)
    return true;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!state_->open)
      return true;
    if (state_->queued_bytes + length > state_->max_queued_bytes) {
      state_->dropped_bytes += length;
      return false;
    }

    // Append to the last chunk if it is still small, otherwise start a new one.
    // The writer thread only ever takes items from the front, so the back item
    // is never one it is working on
    std::deque<Item> &queue = state_->queue;
    if (queue.empty() || queue.back().switch_file ||
        queue.back().data.size() + length > MAX_CHUNK_BYTES)
      queue.emplace_back();
    queue.back().data.insert(queue.back().data.end(), data, data + length);
    state_->queued_bytes += length;
  }
  state_->work_cv.notify_one();
  return true;
}

bool AsyncFileWriter::waitUntilIdle(const std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(state_->mutex);
  return state_->idle_cv.wait_for(lock, timeout, [this]() {
    return state_->queue.empty() && !state_->busy;
  });
}

uint64_t AsyncFileWriter::droppedBytes() const { return state_->dropped_bytes; }

void AsyncFileWriter::run(const std::shared_ptr<State> state) {
  std::unique_ptr<std::ofstream> file;
  std::unique_lock<std::mutex> lock(state->mutex);
  while (true) {
    state->work_cv.wait(
        lock, [&state]() { return state->stop || !state->queue.empty(); });
    if (state->queue.empty())
      break; // Stopping, and everything has been written

    Item item = std::move(state->queue.front());
    state->queue.pop_front();
    state->queued_bytes -= item.data.size();
    state->busy = true;

    // Do the slow part without holding the lock, so producers are never blocked
    // by the disk
    lock.unlock();
    if (item.switch_file) {
      if (file)
        file->close(); // Flushes
      file = std::move(item.file);

      // Now that any previous file is closed, it is safe to discard the old
      // contents. Ignore failures, which happen for things that cannot be
      // truncated, like pipes
      if (file && !item.truncate_path.empty())
        (void)truncate(item.truncate_path.c_str(), 0);
    } else if (file) {
      file->write(reinterpret_cast<const char *>(item.data.data()),
                  static_cast<std::streamsize>(item.data.size()));
      if (!file->good()) {
        // Count the lost data and clear the error, so later writes are
        // attempted again (e.g. once space is freed)
        state->dropped_bytes += item.data.size();
        file->clear();
      }
    }
    lock.lock();

    state->busy = false;
    if (state->queue.empty())
      state->idle_cv.notify_all();
  }
  lock.unlock();

  if (file)
    file->close();

  lock.lock();
  state->finished = true;
  state->idle_cv.notify_all();
}

} // namespace microstrain
