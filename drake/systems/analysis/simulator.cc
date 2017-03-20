#include "drake/systems/analysis/simulator.h"

#include <chrono>
#include <limits>
#include <thread>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace systems {

template <typename T>
void Simulator<T>::PauseIfTooFast() const {
  if (target_realtime_rate_ <= 0) return;  // Run at full speed.
  const double simtime_now = ExtractDoubleOrThrow(get_context().get_time());
  const double simtime_passed = simtime_now - initial_simtime_;
  const TimePoint desired_realtime =
      initial_realtime_ + Duration(simtime_passed / target_realtime_rate_);
  // TODO(sherm1): Could add some slop to now() and not sleep if
  // we are already close enough. But what is a reasonable value?
  if (desired_realtime > Clock::now())
    std::this_thread::sleep_until(desired_realtime);
}

template <typename T>
double Simulator<T>::get_actual_realtime_rate() const {
  const double simtime_now = ExtractDoubleOrThrow(get_context().get_time());
  const double simtime_passed = simtime_now - initial_simtime_;
  const Duration realtime_passed = Clock::now() - initial_realtime_;
  const double rate = (simtime_passed / realtime_passed.count());
  return rate;
}

template <typename T>
void Simulator<T>::ResetStatistics() {
  integrator_->ResetStatistics();
  num_steps_taken_ = 0;
  num_discrete_updates_ = 0;
  num_unrestricted_updates_ = 0;
  num_publishes_ = 0;

  initial_simtime_ = ExtractDoubleOrThrow(get_context().get_time());
  initial_realtime_ = Clock::now();
}

template class Simulator<double>;
template class Simulator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
