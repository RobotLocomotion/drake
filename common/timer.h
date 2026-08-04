#pragma once

// TODO(#16486): Remove chrono include by encapsulating into timer.cc
#include <chrono>

#include "drake/common/drake_copyable.h"

/// @file
/// Provides drake::Timer interface and drake::SteadyTimer for timing events.

namespace drake {

/// Abstract base class for timing utility.
class Timer {
 public:
  /// Properly implemented Timers must start timing upon construction.
  Timer() = default;

  virtual ~Timer();

  /// Begins timing. Call Start every time you want to reset the timer to zero.
  virtual void Start() = 0;

  /// Obtains a timer measurement in seconds.
  /// Call this repeatedly to get multiple measurements.
  /// @return the amount of time since the timer started.
  virtual double Tick() = 0;

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Timer);  // protected from slicing
};

/// Implementation of timing utility that uses monotonic
/// std::chrono::steady_clock.
class SteadyTimer final : public Timer {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SteadyTimer);
  SteadyTimer();
  ~SteadyTimer() final;
  void Start() final;
  double Tick() final;

 private:
  using clock = std::chrono::steady_clock;
  clock::time_point start_time_;
};

/// Implementation of timing for use with unit tests that control time manually.
class ManualTimer final : public Timer {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ManualTimer);
  ManualTimer();
  ~ManualTimer() final;
  void Start() final;
  double Tick() final;

  /// Sets the return value of Tick().
  void set_tick(double tick) { tick_ = tick; }

 private:
  double tick_{};
};

}  // namespace drake
