#pragma once

#include <chrono>  // TODO(#16981) Remove chrono include by encapsulating into timer.cc

#include "common/drake_copyable.h"

/// @file
/// Provides drake::Timer interface and drake::SteadyTimer for timing events.

namespace drake {

/// Abstract base class for timing utility.
struct Timer {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Timer)

  /// Timers start upon construction for convenience.
  Timer() = default;
  virtual ~Timer() = default;

  /// Begins timing. Call Start everytime you want to reset the timer to zero.
  virtual void Start() = 0;

  /// Obtains a timer measurement in seconds.
  /// Call this repeatedly to get multiple measurements.
  /// @return the amount of time since the timer started.
  virtual double Tick() = 0;
};

/// Implementation of timing utility that uses monotonic
/// std::chrono::steady_clock.
class SteadyTimer : public Timer {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SteadyTimer)
  SteadyTimer();
  using clock = std::chrono::steady_clock;
  void Start() override;
  double Tick() override;

 protected:
  clock::time_point start_time_;
};

}  // namespace drake
