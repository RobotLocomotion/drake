#pragma once

#include <algorithm>
#include <condition_variable>
#include <mutex>

namespace drake {
namespace systems {
namespace lcm {

/**
 * A sempahore based on std::mutex and std::condition_variable. This class
 * holds an internal counter, which is incremented by one up to a max limit
 * every time notify() is called, and decremented by one every time wait() is
 * called.
 */
class Semaphore {
 public:
  /**
   * Constructor.
   * @param max_count Max counter limit.
   */
  explicit Semaphore(uint64_t max_count) : max_count_(max_count) {}

  /**
   * Wakes up one thread that is sleeping on this. Increment the counter by
   * one up to max counter limit.
   */
  void notify() {
    std::unique_lock<std::mutex> lock(mutex_);
    count_++;
    count_ = std::min(count_, max_count_);
    condition_.notify_one();
  }

  /**
   * The caller blocks until some other thread calls notify(). Upon wake up,
   * the internal counter is decremented by one.
   */
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

}  // namespace lcm
}  // namespace systems
}  // namespace drake
