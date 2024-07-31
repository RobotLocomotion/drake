#pragma once

#include <memory>
#include <optional>

#include "drake/common/timer.h"

namespace drake {
namespace systems {
namespace internal {

/* Computes real time rate(s) over a pre-determined "report" period of wall
 clock time. By limiting calculations to a fixed period (throttling), we
 inherently smooth the signal. Using code should call UpdateAndRecalculation()
 to inform the calculator of simulation time advancement. The calculator's
 report accounts for _every_ elapsed report period. If multiple report periods
 elapse between invocations of UpdateAndRecalculation(), the reported rate will
 be associated with each of those periods (see ReportRate::period_count). */
class RealtimeRateCalculator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RealtimeRateCalculator);

  /* The result of updating current sim time (and possibly computing a realtime
   rate). The result will suggest one of three possible outcomes:
   "initialization", "incomplete", or "computed". The three values are
   encoded as:

     - initialization: `initialized` is `true`. `period_count` and `rate` should
       be ignored.
     - incomplete: `initialized` is `false` and `period_count` is 0. `rate`
       should be ignored.
     - computed rate: `initialized` is `false`, `period_count` > 0, and `rate`
       must be used by the caller.

   Note: A positive value of `period_count` indicates how many "report periods"
   have passed since the last calculation of the rate. Each period shares the
   same `rate` value. */
  struct UpdateResult {
    /* Constructs an instance representing the "initialized" value. */
    static UpdateResult Initialized();

    /* Constructs an instance representing the "incomplete" value. */
    static UpdateResult Incomplete();

    /* Constructs an instance representing the "computed" value. */
    static UpdateResult Computed(int period_count, double rate);

    /* `true` if the calculator was initialized on this call. */
    bool initialized{};

    /* The number of periods to which the `rate` value applies. Zero indicates
     no value calculated. If `initialized` is true, this value is undefined. */
    int period_count{};

    /* The computed rate. If `period_count` is zero, or `initialized` is false,
     this value is undefined. */
    double rate{};
  };

  /* Constructs the calculator with the given `report_period`.

   @param report_period       The duration of the report period (in wall clock
                              seconds). Must be positive. */
  explicit RealtimeRateCalculator(double report_period);

  /* Updates the calculator's knowledge of the current simulation time and
   computes a resulting realtime rate, where possible.

   The returned result will have the "initialized" value if:

     - this is the first invocation, or
     - `current_sim_time` is strictly less than cached sim time at the last
       successful rate calculation. That value is not necessarily the same as
       the value of `current_sim_time` of the previous invocation.
       - See the Reset() method for an alternative method for resetting the
         calculator and re-initializing.

   The returned result will have the "incomplete" value if:

     - a previous call initialized the calculator, and
     - less than `report_period` seconds have passed since the last successful
       rate calculation.

   Otherwise, the returned result will have the "computed" value.

   Depending on how much wall time has passed, the calculator may find that
   multiple report periods have elapsed. The result's `period_count` value will
   contain the positive count of the number of periods that have passed. The
   corresponding rate value applies to each of those periods. */
  UpdateResult UpdateAndRecalculate(double current_sim_time);

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
  void InjectMockTimer(std::unique_ptr<Timer>);

 private:
  bool initialized_{false};
  const double period_;
  std::unique_ptr<Timer> timer_{std::make_unique<SteadyTimer>()};

  double prev_sim_time_{};
  double prev_wall_time_{};
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
