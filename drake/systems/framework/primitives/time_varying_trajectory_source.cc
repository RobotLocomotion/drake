#include "time_varying_trajectory_source.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
TimeVaryingTrajectorySource<T>::TimeVaryingTrajectorySource(
    const Trajectory& trajectory)
    : trajectory_(trajectory) {
  this->DeclareOutputPort(kVectorValued, trajectory_.length(),
                          kContinuousSampling);
}

template <typename T>
void TimeVaryingTrajectorySource<T>::EvalOutput(
    const Context<T>& context, SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  T time = context.get_time();
  System<T>::GetMutableOutputVector(output, 0) = trajectory_.value(time);
}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_EXPORT TimeVaryingTrajectorySource<double>;

}  // namespace systems
}  // namespace drake
