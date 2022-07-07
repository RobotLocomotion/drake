#include "drake/systems/analysis/instantaneous_realtime_rate_calculator.h"

#include <utility>

namespace drake {
namespace systems {
namespace internal {

std::optional<double>
InstantaneousRealtimeRateCalculator::CalculateRealtimeRate(
    double current_sim_time) {
  std::optional<double> realtime_rate;
  if (prev_sim_time_.has_value()) {
    const double wall_delta{timer_->Tick()};
    const double sim_time_delta{current_sim_time - prev_sim_time_.value()};
    // Avoid divide by zero and negative rate.
    if (wall_delta > 0 && sim_time_delta >= 0) {
      realtime_rate = sim_time_delta / wall_delta;
    }
  }
  timer_->Start(); // Restarts the wall timer
  prev_sim_time_ = current_sim_time;
  return realtime_rate;
}

void InstantaneousRealtimeRateCalculator::InjectMockTimer(
    std::unique_ptr<Timer> mock_timer) {
  timer_ = std::move(mock_timer);
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
