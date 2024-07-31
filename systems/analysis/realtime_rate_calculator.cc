#include "drake/systems/analysis/realtime_rate_calculator.h"

#include <cmath>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace internal {

using UpdateResult = RealtimeRateCalculator::UpdateResult;

UpdateResult UpdateResult::Initialized() {
  return {.initialized = true};
}

UpdateResult UpdateResult::Incomplete() {
  return {.initialized = false, .period_count = 0};
}

UpdateResult UpdateResult::Computed(int period_count, double rate) {
  return {.initialized = false, .period_count = period_count, .rate = rate};
}

RealtimeRateCalculator::RealtimeRateCalculator(double report_period)
    : period_(report_period) {
  DRAKE_DEMAND(report_period > 0);
}

UpdateResult RealtimeRateCalculator::UpdateAndRecalculate(
    double current_sim_time) {
  if (std::isnan(current_sim_time)) {
    return UpdateResult::Incomplete();
  }

  if (!initialized_) {
    prev_sim_time_ = current_sim_time;
    prev_wall_time_ = 0.0;
    timer_->Start();
    initialized_ = true;
    return UpdateResult::Initialized();
  }

  // sim_delta will only be NaN if either operand is Nan (subtraction won't
  // create a NaN from non-NaN values). We've already confirmed current_sim_time
  // is not NaN and prev_sim_time_'s definition won't introduce NaN.
  const double sim_delta = current_sim_time - prev_sim_time_;
  if (sim_delta < 0) {
    // Moving backwards reinitializes the calculation.
    prev_sim_time_ = current_sim_time;
    prev_wall_time_ = 0.0;
    timer_->Start();
    return UpdateResult::Initialized();
  } else if (sim_delta == 0) {
    // Repeated simulation times do nothing.
    return UpdateResult::Incomplete();
  }

  const double wall_delta = timer_->Tick() - prev_wall_time_;
  // Time should always be monotonically increasing; this puts a small,
  // reasonable constraint on unit tests (when using ManualTimer).
  DRAKE_DEMAND(wall_delta >= 0 && std::isfinite(wall_delta));
  if (wall_delta < period_) {
    return UpdateResult::Incomplete();
  }

  const double rate = sim_delta / wall_delta;

  // Note: we don't necessarily store the latest passed simulation time nor the
  // wall clock time provided by our timer. Instead, we store the wall time at
  // the nearest previous period boundary and an approximation of the sim
  // time advancement at that period boundary wall clock time.
  const int period_count = std::floor(wall_delta / period_);
  // We're only here because wall_delta > period_.
  DRAKE_DEMAND(period_count > 0);
  const double time_advance = period_ * period_count;
  prev_wall_time_ += time_advance;
  // Note: we only accumulate fractional simulation time if both previous and
  // current sim times are *finite*. Otherwise, we assume that all of the
  // advances to simulation time belonged to the previous rate calculation.
  // (This comes from setting simulation time to negative infinity as a means
  // to do a hard reset knowing that almost any other double value for
  // simulation time represents an advancement in time.)
  if (std::isfinite(prev_sim_time_) && std::isfinite(sim_delta)) {
    const double fraction = time_advance / wall_delta;
    prev_sim_time_ += fraction * sim_delta;
  } else {
    prev_sim_time_ = current_sim_time;
  }
  return UpdateResult::Computed(period_count, rate);
}

void RealtimeRateCalculator::InjectMockTimer(
    std::unique_ptr<Timer> mock_timer) {
  timer_ = std::move(mock_timer);
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
