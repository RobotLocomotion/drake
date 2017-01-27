#include "drake/systems/primitives/trajectory_source.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
TrajectorySource<T>::TrajectorySource(const Trajectory& trajectory)
    : SingleOutputVectorSource<T>(trajectory.rows()),
      trajectory_(trajectory) {
  // This class does not currently support trajectories which output
  // more complicated matrices.
  DRAKE_DEMAND(trajectory.cols() == 1);
}

template <typename T>
void TrajectorySource<T>::DoCalcVectorOutput(
    const Context<T>& context, Eigen::VectorBlock<VectorX<T>>* output) const {
  *output = trajectory_.value(context.get_time());
}

// Explicitly instantiates on the most common scalar types.
template class TrajectorySource<double>;

}  // namespace systems
}  // namespace drake
