#include "drake/systems/primitives/trajectory_source.h"

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/ssize.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {

using trajectories::Trajectory;

template <typename T>
TrajectorySource<T>::TrajectorySource(const Trajectory<T>& trajectory,
                                      int output_derivative_order,
                                      bool zero_derivatives_beyond_limits)
    : SingleOutputVectorSource<T>(
          SystemTypeTag<TrajectorySource>{},
          trajectory.rows() * (1 + output_derivative_order)),
      // Make a copy of the input trajectory.
      trajectory_(trajectory.Clone()),
      clamp_derivatives_(zero_derivatives_beyond_limits) {
  // This class does not currently support trajectories which output
  // more complicated matrices.
  DRAKE_DEMAND(trajectory.cols() == 1);
  DRAKE_DEMAND(output_derivative_order >= 0);

  for (int i = 0; i < output_derivative_order; ++i) {
    if (i == 0)
      derivatives_.push_back(trajectory_->MakeDerivative());
    else
      derivatives_.push_back(derivatives_[i - 1]->MakeDerivative());
  }
}

template <typename T>
template <typename U>
TrajectorySource<T>::TrajectorySource(const TrajectorySource<U>& other)
    : SingleOutputVectorSource<T>(
          SystemTypeTag<TrajectorySource>{},
          other.trajectory_->rows() * (1 + ssize(other.derivatives_))),
      clamp_derivatives_{other.clamp_derivatives_} {
  if constexpr (std::is_same_v<U, double>) {
    failsafe_trajectory_ = other.trajectory_->Clone();
    for (int i = 0; i < ssize(other.derivatives_); ++i) {
      failsafe_derivatives_.emplace_back(other.derivatives_[i]->Clone());
    }
  } else {
    throw std::runtime_error(
        "TrajectorySource only supports scalar conversion from double.");
  }
}

template <typename T>
void TrajectorySource<T>::UpdateTrajectory(
    const trajectories::Trajectory<T>& trajectory) {
  DRAKE_DEMAND(trajectory_ != nullptr || failsafe_trajectory_ != nullptr);
  int rows = trajectory_ ? trajectory_->rows() : failsafe_trajectory_->rows();
  DRAKE_DEMAND(trajectory.rows() == rows);
  DRAKE_DEMAND(trajectory.cols() == 1);

  trajectory_ = trajectory.Clone();
  for (int i = 0; i < static_cast<int>(derivatives_.size()); ++i) {
    if (i == 0)
      derivatives_[i] = trajectory_->MakeDerivative();
    else
      derivatives_[i] = derivatives_[i - 1]->MakeDerivative();
  }
  failsafe_trajectory_ = nullptr;
  failsafe_derivatives_.clear();
}

template <typename T>
void TrajectorySource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  DRAKE_DEMAND(trajectory_ != nullptr || failsafe_trajectory_ != nullptr);
  int len = trajectory_ ? trajectory_->rows() : failsafe_trajectory_->rows();
  T time = context.get_time();
  double maybe_time_as_double{std::numeric_limits<double>::quiet_NaN()};
  if (trajectory_ != nullptr) {
    output->head(len) = trajectory_->value(time);
  } else {
    // TODO(russt): We are missing DiscardZeroGradient for scalars, and/or
    // a version of ExtractDoubleOrThrow which throws on non-zero gradients.
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      if (time.derivatives().size() != 0 && !time.derivatives().isZero()) {
        throw std::runtime_error(
            "Cannot take gradients w.r.t. time with this TrajectorySource. You "
            "must call UpdateTrajectory() to provide a "
            "Trajectory<AutoDiffXd>.");
      }
    }
    maybe_time_as_double = ExtractDoubleOrThrow(context.get_time());
    output->head(len) = failsafe_trajectory_->value(maybe_time_as_double);
  }

  T start_time = trajectory_ ? trajectory_->start_time()
                             : failsafe_trajectory_->start_time();
  T end_time =
      trajectory_ ? trajectory_->end_time() : failsafe_trajectory_->end_time();
  boolean<T> set_zero =
      boolean<T>(clamp_derivatives_) && (time > end_time || time < start_time);

  DRAKE_DEMAND(derivatives_.size() == 0 || failsafe_derivatives_.size() == 0);
  for (int i = 0; i < ssize(derivatives_); ++i) {
    VectorX<T> value = derivatives_[i]->value(context.get_time());
    for (int j = 0; j < len; ++j) {
      (*output)[len * (i + 1) + j] = if_then_else(set_zero, T(0), value[j]);
    }
  }
  for (int i = 0; i < ssize(failsafe_derivatives_); ++i) {
    DRAKE_DEMAND(!std::isnan(maybe_time_as_double));
    VectorX<T> value = failsafe_derivatives_[i]->value(maybe_time_as_double);
    for (int j = 0; j < len; ++j) {
      (*output)[len * (i + 1) + j] = if_then_else(set_zero, T(0), value[j]);
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TrajectorySource)
