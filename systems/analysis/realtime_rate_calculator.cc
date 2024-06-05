#include "drake/systems/analysis/realtime_rate_calculator.h"

#include <cmath>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace internal {

RealtimeRateCalculator::RealtimeRateCalculator(double report_period)
    : period_(report_period) {
  DRAKE_DEMAND(report_period > 0);
}

RealtimeRateCalculator::RateReport RealtimeRateCalculator::UpdateAndRecalculate(
    double current_sim_time) {
  if (std::isnan(current_sim_time)) {
    return {.initialized = false, .period_count = 0};
  }

  if (!initialized_) {
    prev_sim_time_ = 0.0;
    prev_wall_time_ = 0.0;
    timer_->Start();
    initialized_ = true;
    return {.initialized = true};
  }

  const double sim_delta = current_sim_time - prev_sim_time_;
  if (!(sim_delta > 0)) {
    bool required_initialization = false;
    if (sim_delta < 0) {
      // Moving backwards resets the calculation.
      prev_sim_time_ = current_sim_time;
      prev_wall_time_ = 0.0;
      timer_->Start();
      required_initialization = true;
    }
    // Whether re-initializing or simply not advancing, we report no rate.
    return {.initialized = required_initialization, .period_count = 0};
  }

  const double wall_delta = timer_->Tick() - prev_wall_time_;
  if (wall_delta < period_) {
    return {.initialized = false, .period_count = 0};
  }

  // Note: we store previous *wall* time at the largest period boundary less
  // than the current time. We store previous *sim* time based on the
  // interpolated simulation time that would've been reached at that boundary.
  const double rate = sim_delta / wall_delta;
  const int period_count = std::floor(wall_delta / period_);
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
  return {.initialized = false, .period_count = period_count, .rate = rate};
}

void RealtimeRateCalculator::InjectMockTimer(
    std::unique_ptr<Timer> mock_timer) {
  timer_ = std::move(mock_timer);
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
