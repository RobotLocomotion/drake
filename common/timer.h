#pragma once

#include <chrono>

/// @file
/// Provides drake::Timer interface and drake::SteadyTimer for timing events

namespace drake {

/// Abstract base class for timing utility.
struct Timer {
  using duration = std::chrono::duration<double>;
  virtual ~Timer() = default;
  /// begin timing
  virtual void Start() = 0;
  /// stop timing
  /// @return the ammount of time since the timer started
  virtual duration Stop() = 0;
  /// obtain multiple measurements from the same baseline
  /// @return the ammount of time since the timer started
  virtual duration Tick() = 0;
};

/// Implementation of timing utility that uses monotonic
/// std::chrono::steady_clock
class SteadyTimer : public Timer {
 public:
  using clock = std::chrono::steady_clock;
  void Start() override { start_time = clock::now(); }
  duration Tick() override { return duration(clock::now() - start_time); }
  duration Stop() override { return Tick(); }

 protected:
  clock::time_point start_time{clock::now()};
};

}  // namespace drake
