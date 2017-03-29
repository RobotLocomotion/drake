#pragma once

#include <algorithm>
#include <condition_variable>
#include <mutex>

class Semaphore {
 public:
  explicit Semaphore(uint64_t max_count) : max_count_(max_count) {}

  void notify() {
    std::unique_lock<std::mutex> lock(mutex_);
    count_++;
    count_ = std::min(count_, max_count_);
    condition_.notify_one();
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    // Handles spurious wake-ups.
    while (!count_)
      condition_.wait(lock);
    count_--;
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  uint64_t count_{0};
  const uint64_t max_count_{1};
};
