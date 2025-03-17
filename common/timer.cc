#include "drake/common/timer.h"

namespace drake {

Timer::~Timer() = default;

SteadyTimer::SteadyTimer() {
  Start();
}

SteadyTimer::~SteadyTimer() = default;

void SteadyTimer::Start() {
  start_time_ = clock::now();
}

double SteadyTimer::Tick() {
  return std::chrono::duration<double>(clock::now() - start_time_).count();
}

ManualTimer::ManualTimer() {
  Start();
}

ManualTimer::~ManualTimer() = default;

void ManualTimer::Start() {
  tick_ = 0.0;
}

double ManualTimer::Tick() {
  return tick_;
}

}  // namespace drake
