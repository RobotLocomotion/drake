#include "drake/systems/primitives/trajectory_source.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

using trajectories::Trajectory;

template <typename T>
TrajectorySource<T>::TrajectorySource(const Trajectory<T>& trajectory,
                                      int output_derivative_order,
                                      bool zero_derivatives_beyond_limits)
    : SingleOutputVectorSource<T>(trajectory.rows() *
                                  (1 + output_derivative_order)),
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
void TrajectorySource<T>::UpdateTrajectory(
    const trajectories::Trajectory<T>& trajectory) {
  DRAKE_DEMAND(trajectory.rows() == trajectory_->rows());
  DRAKE_DEMAND(trajectory.cols() == 1);

  trajectory_ = trajectory.Clone();
  for (int i = 0; i < static_cast<int>(derivatives_.size()); ++i) {
    if (i == 0)
      derivatives_[i] = trajectory_->MakeDerivative();
    else
      derivatives_[i] = derivatives_[i - 1]->MakeDerivative();
  }
}

template <typename T>
void TrajectorySource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  int len = trajectory_->rows();
  output->head(len) = trajectory_->value(context.get_time());

  T time = context.get_time();
  bool set_zero = false;
  if (clamp_derivatives_ && !scalar_predicate<T>::is_bool) {
    // zero_derivatives_beyond_limits is true by default, but presumably most
    // users will not want the clamped derivatives for symbolic types.
    log()->warn(
        "TrajectorySource: Derivatives are not clamped for symbolic types. "
        "Pass zero_derivatives_beyond_limits=false in the constructor to avoid "
        "this warning");
  } else {
    set_zero = clamp_derivatives_ && (time > trajectory_->end_time() ||
                                      time < trajectory_->start_time());
  }

  for (size_t i = 0; i < derivatives_.size(); ++i) {
    if (set_zero) {
      output->segment(len * (i + 1), len).setZero();
    } else {
      output->segment(len * (i + 1), len) =
          derivatives_[i]->value(context.get_time());
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TrajectorySource)
