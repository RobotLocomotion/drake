#pragma once

#include <memory>
#include <optional>

#include "drake/common/timer.h"

namespace drake {
namespace systems {
namespace internal {

/* Computes real time rate(s) over a pre-determined period. By throttling the
 calculation, we inherently smooth the signal. For a given report period `r`,
 the calculator will compute a unique value for wall clock times `r·k` (for
 k ∈ ℤ). Repeated calls to UpdateAndRecalculation() inform the calculator of
 the progression of simulation time, causing one or more calculations to be
 returned. */
class RealtimeRateCalculator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RealtimeRateCalculator);

  /* Constructs the calculator with the given `report_period`.

   @param report_period       The duration of the report period (in wall clock
                              seconds). Must be positive. */
  explicit RealtimeRateCalculator(double report_period);

  /* The result of updating current sim time. If no rate could be computed,
   `period_count` is equal to zero. For a positive value of `period_count`, it
   indicates how many "report periods" have passed since the last calculation
   of the rate. Each period shares the same `rate` value. */
  struct RateReport {
    /* The computed rate. If `period_count` is zero, or `initialized` is false,
     this value is undefined. */
    double rate{};

    /* The number of periods to which the `rate` value applies. Zero indicates
     no value calculated. */
    int period_count{};

    /* `true` if the calculator was initialized on this call. */
    bool initialized{};
  };

  /* Updates the calculator's knowledge of the current simulation time.

   Depending on how much wall time has passed, the calculator may find that the
   report period boundary hasn't been reached. Conversely, it may find that
   multiple report periods have elapsed. The return value's `period_count`
   field communicates that state.

     - 0:  Since initialization or last report period boundary, not enough time
           has elapsed and no rate can be computed.
     - >0: With this update, we've crossed `period_count` number of report
           period boundaries. The `rate` value is the rate value for each of
           those periods.

   `period_count` will be zero in the following cases:

     - First invocation (either after construction or calling Reset()).
     - `current_sim_time` is *equal* to the value passed on the previous
       invocation.
     - `current_sim_time` is *strictly less* than the largest value previously
       passed (since last initialization). Furthermore, this resets the
       calculator to the given `current_sim_time` and the wall clock value at
       the time of invocation.
     - The wall clock hasn't advanced at least `report_period` seconds since the
       last reported rate.

   See Reset() for an alternative method for resetting the calculator. */
  RateReport UpdateAndRecalculate(double current_sim_time);

  /* Resets the internal state of `this` calculator. After a call, the next call
   to UpdateAndRecalculate() will re-seed the rate calculation as if it were the
   first call.
   
   This is subtly different from calling UpdateAndRecalculate() with a smaller
   simulation time. That act *initializes* the calculator such that the next
   invocation can compute a rate (assuming enough wall clock time has passed).
   Calling this method resets the whole calculator such that it is waiting for
   the first call to initialize it. */
  void Reset() { initialized_ = false; }

  /* (Internal use for unit testing only) Used to mock the monotonic wall time
   source to control time during unit testing.  */
#ifndef DRAKE_DOXYGEN_CXX
  void InjectMockTimer(std::unique_ptr<Timer>);
#endif

 private:
  bool initialized_{false};
  const double period_{};
  std::unique_ptr<Timer> timer_{std::make_unique<SteadyTimer>()};

  double prev_sim_time_{};
  double prev_wall_time_{};
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
