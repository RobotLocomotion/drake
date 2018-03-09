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

  for (int i = 0; i < output_derivative_order; i++) {
    if (i == 0)
      derivatives_.push_back(trajectory_->MakeDerivative());
    else
      derivatives_.push_back(derivatives_[i - 1]->MakeDerivative());
  }
}

template <typename T>
void TrajectorySource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  int len = trajectory_->rows();
  output->head(len) = trajectory_->value(context.get_time());

  double time = context.get_time();
  bool set_zero = clamp_derivatives_ && (time > trajectory_->end_time() ||
      time < trajectory_->start_time());

  for (size_t i = 0; i < derivatives_.size(); ++i) {
    if (set_zero) {
      output->segment(len * (i + 1), len).setZero();
    } else {
      output->segment(len * (i + 1), len) =
          derivatives_[i]->value(context.get_time());
    }
  }
}

template class TrajectorySource<double>;

}  // namespace systems
}  // namespace drake
