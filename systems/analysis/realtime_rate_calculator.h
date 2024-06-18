#pragma once

#include <memory>
#include <optional>

#include "drake/common/timer.h"

namespace drake {
namespace systems {
namespace internal {
/* Utility class that computes the realtime rate achieved between time steps. */
class RealtimeRateCalculator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RealtimeRateCalculator);
  RealtimeRateCalculator() = default;

  /* Computes the realtime rate which is the ratio of the amount of simulator
   time to real world time that has passed between invocations.
   The very first call to this function seeds the rate calculation and returns
   nullopt because a valid rate cannot be computed yet. It will also return
   nullopt if sim_time goes backwards.
    @param current_sim_time the current simulated time.
    @return realtime rate if one can be calculated, nullopt otherwise.
   */
  std::optional<double> UpdateAndRecalculate(double current_sim_time);

  /* Resets the internal state of `this` rate calculator. After a call, the next
   call to UpdateAndRecalculate() will re-seed the rate calculation as if it was
   the first call.
   */
  void Reset() { prev_sim_time_ = std::nullopt; }

  /* (Internal use for unit testing only) Used to mock the monotonic wall time
     source to control time during unit testing.  */
#ifndef DRAKE_DOXYGEN_CXX
  void InjectMockTimer(std::unique_ptr<Timer>);
#endif

 private:
  std::optional<double> prev_sim_time_;
  std::unique_ptr<Timer> timer_{std::make_unique<SteadyTimer>()};
};

/* Calculates real time rate on a period basis. Various measurements can be
 taken in sequence (measuring sim time and the corresponding wall clock time)
 and, periodically, this calculator can be asked for the average real time rate
 of the period since last request. */
class PeriodicRealtimeRateCalculator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PeriodicRealtimeRateCalculator);

  /* Constructs the calculator. The calculator can report realtime rate with
   either a simulation time period or a wall clock time period.

   @param report_period       The duration of the report period (in seconds).
   @param period_in_sim_time  If `true`, `report_period` is interpreted as
                              simulation time. */
  PeriodicRealtimeRateCalculator(double report_period, bool period_in_sim_time);

  /* When updating/calculating the */
  struct RateReport {
    double rate;
    int period_count;
  };

  /* Updates the calculator's knowledge of the current simulation time.
   Depending on how much time has passed, the calculator may find that the
   report period boundary hasn't been reached. Conversely, it may find that
   multiple report periods have elapsed. This return value's `period_count`
   field communicates that state.

     - 0: Since initialization or last report period boundary, not enough time
          has elapsed and no rate can be computed.
     - >0: With this update, we've crossed `period_count` number of report
           period boundaries. The `rate` value represents the rate computed for
           the *first* of those report periods. However, there are additional
           periods that need to be reported. Callers should make repeated calls
           to this method, with the same value of `current_sim_time` to retrieve
           the computed rates for each pending report period. */
  RateReport UpdateAndRecalculate(double current_sim_time);

  /* Resets the internal state of `this` calculator. After a call, the next call
   to UpdateAndRecalculate() will re-seed the rate calculation as if it were the
   first call. */
  void Reset() { initialized_ = false; }

  /* (Internal use for unit testing only) Used to mock the monotonic wall time
     source to control time during unit testing.  */
#ifndef DRAKE_DOXYGEN_CXX
  void InjectMockTimer(std::unique_ptr<Timer>);
#endif

 private:
  /* Does the actual work for UpdateAndRecalculate() if `period_in_sim_` is
   `true`. */
  RateReport CalcForSimPeriod(double current_sim_time);

  /* Does the actual work for UpdateAndRecalculate() if `period_in_sim_` is
   `false`. */
  RateReport CalcForWallPeriod(double current_sim_time);

  bool initialized_{false};
  const double period_{};
  const bool period_in_sim_{};
  std::unique_ptr<Timer> timer_{std::make_unique<SteadyTimer>()};

  int next_step_{0};
  double prev_sim_time_{};
  double prev_wall_time_{};
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
