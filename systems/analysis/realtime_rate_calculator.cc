#include "drake/systems/analysis/realtime_rate_calculator.h"

#include <cmath>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace internal {

std::optional<double>
RealtimeRateCalculator::UpdateAndRecalculate(double current_sim_time) {
  std::optional<double> realtime_rate;
  if (prev_sim_time_.has_value()) {
    const double wall_delta{timer_->Tick()};
    const double sim_time_delta{current_sim_time - prev_sim_time_.value()};
    // Avoid divide by zero and negative rate.
    if (wall_delta > 0 && sim_time_delta >= 0) {
      realtime_rate = sim_time_delta / wall_delta;
    }
  }
  timer_->Start();  // Restarts the wall timer
  prev_sim_time_ = current_sim_time;
  return realtime_rate;
}

void RealtimeRateCalculator::InjectMockTimer(
    std::unique_ptr<Timer> mock_timer) {
  timer_ = std::move(mock_timer);
}

PeriodicRealtimeRateCalculator::PeriodicRealtimeRateCalculator(
    double report_period, bool period_in_sim_time)
    : period_(report_period), period_in_sim_(period_in_sim_time) {}

PeriodicRealtimeRateCalculator::RateReport
PeriodicRealtimeRateCalculator::UpdateAndRecalculate(double current_sim_time) {
  if (!initialized_) {
    next_step_ = 1;
    prev_sim_time_ = 0.0;
    prev_wall_time_ = 0.0;
    timer_->Start();
    initialized_ = true;
    return {0, 0};
  }

  if (period_in_sim_) {
    return CalcForSimPeriod(current_sim_time);
  } else {
    return CalcForWallPeriod(current_sim_time);
  }
}

void PeriodicRealtimeRateCalculator::InjectMockTimer(
    std::unique_ptr<Timer> mock_timer) {
  timer_ = std::move(mock_timer);
}

PeriodicRealtimeRateCalculator::RateReport
PeriodicRealtimeRateCalculator::CalcForSimPeriod(double current_sim_time) {
  const double sim_delta = current_sim_time - prev_sim_time_;
  if (sim_delta < period_) {
    return {0, 0};
  }
  const double wall_delta = timer_->Tick() - prev_wall_time_;
  DRAKE_DEMAND(wall_delta > 0);

  const double rate = sim_delta / wall_delta;
  prev_sim_time_ = next_step_ * period_;
  ++next_step_;
  prev_wall_time_ += (period_ / sim_delta) * wall_delta;
  const int period_count = std::floor(sim_delta / period_);
  return {rate, period_count};
}

PeriodicRealtimeRateCalculator::RateReport
PeriodicRealtimeRateCalculator::CalcForWallPeriod(double current_sim_time) {
  const double wall_delta = timer_->Tick() - prev_wall_time_;
  DRAKE_DEMAND(wall_delta > 0);

  if (wall_delta < period_) {
    return {0, 0};
  }

  const double sim_delta = current_sim_time - prev_sim_time_;
  if (sim_delta <= 0) {
    // We have no report of simulation time advancing in the report period, so
    // we *can't* report a rate.
    return {0, 0};
  }

  const double rate = sim_delta / wall_delta;
  prev_wall_time_ = period_ * next_step_;
  ++next_step_;
  prev_sim_time_ += (period_ / wall_delta) * sim_delta;
  const int period_count = std::floor(wall_delta / period_);
  return {rate, period_count};
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
