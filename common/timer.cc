#include "common/timer.h"

namespace drake {

SteadyTimer::SteadyTimer() : start_time_{clock::now()} {}

void SteadyTimer::Start() { start_time_ = clock::now(); }

SteadyTimer::duration SteadyTimer::Tick() {
  return {clock::now() - start_time_};
}

}  // namespace drake
