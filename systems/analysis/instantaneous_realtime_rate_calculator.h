#pragma once

#include <memory>
#include <optional>

#include "drake/common/timer.h"

namespace drake {
namespace systems {
/// Utility class that computes the realtime rate achieved between time steps.
class InstantaneousRealtimeRateCalculator {
 public:
  /** Computes the realtime rate between time steps. The very first call to this
   function seeds the rate calculation and returns an empty optional because a
   valid rate cannot be computed yet. It will also return an empty optional if
   sim_time goes backwards.
    @param current_sim_time the current simulated time.
    @return realtime rate if one can be calculated, empty optional otherwise.
   */
  std::optional<double> CalculateRealtimeRate(double current_sim_time);

  /* (Internal use for unit testing only) Used to mock the monatomic wall time
     source to control time during unit testing.  */
#ifndef DRAKE_DOXYGEN_CXX
  void InjectMockTimer(std::unique_ptr<Timer>);
#endif

 private:
  std::optional<double> prev_sim_time_{};
  std::unique_ptr<Timer> timer_{std::make_unique<SteadyTimer>()};
};
}  // namespace systems
}  // namespace drake