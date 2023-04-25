#include "common/timer.h"

namespace drake {

SteadyTimer::SteadyTimer() {
  Start();
}

void SteadyTimer::Start() {
  start_time_ = clock::now();
}

double SteadyTimer::Tick() {
  return std::chrono::duration<double>(clock::now() - start_time_).count();
}

}  // namespace drake
