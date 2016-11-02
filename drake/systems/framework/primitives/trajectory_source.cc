#include "drake/systems/framework/primitives/trajectory_source.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
TrajectorySource<T>::TrajectorySource(
    const Trajectory& trajectory)
    : trajectory_(trajectory) {
  this->DeclareOutputPort(kVectorValued, trajectory_.rows(),
                          kContinuousSampling);
  // This class does not currently support trajectories which output
  // more complicated matrices.
  DRAKE_DEMAND(trajectory.cols() == 1);
}

template <typename T>
void TrajectorySource<T>::EvalOutput(
    const Context<T>& context, SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  T time = context.get_time();
  System<T>::GetMutableOutputVector(output, 0) = trajectory_.value(time);
}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_EXPORT TrajectorySource<double>;

}  // namespace systems
}  // namespace drake
