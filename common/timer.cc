#include "common/timer.h"

namespace drake {

SteadyTimer::SteadyTimer() : start_time{clock::now()} {}

void SteadyTimer::Start() { start_time = clock::now(); }

SteadyTimer::duration SteadyTimer::Tick() {
  return {clock::now() - start_time};
}

}  // namespace drake
