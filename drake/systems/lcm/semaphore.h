#pragma once
#include <mutex>
#include <condition_variable>
#include <iostream>

class Semaphore {
 public:
  explicit Semaphore(unsigned long max_count) : max_count_(max_count) {}

  void notify() {
    std::unique_lock<std::mutex> lock(mutex_);
    count_++;
    count_ = std::min(count_, max_count_);
    condition_.notify_one();
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    // Handle spurious wake-ups.
    while (!count_)
      condition_.wait(lock);
    count_--;
  }

  bool try_wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (count_) {
      count_--;
      return true;
    }
    return false;
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  unsigned long count_{0};
  const unsigned long max_count_{1};
};
