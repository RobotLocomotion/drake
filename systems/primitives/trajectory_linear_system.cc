#include "drake/systems/primitives/trajectory_linear_system.h"

namespace drake {
namespace systems {

template <typename T>
TrajectoryLinearSystem<T>::TrajectoryLinearSystem(
    const trajectories::Trajectory<double>& A,
    const trajectories::Trajectory<double>& B,
    const trajectories::Trajectory<double>& C,
    const trajectories::Trajectory<double>& D,
    double time_period)
    : TimeVaryingLinearSystem<T>(
          SystemTypeTag<TrajectoryLinearSystem>{}, A.rows(),
          B.cols(), C.rows(), time_period),
      A_(A.Clone()),
      B_(B.Clone()),
      C_(C.Clone()),
      D_(D.Clone()) {}

template <typename T>
template <typename U>
TrajectoryLinearSystem<T>::TrajectoryLinearSystem(
    const TrajectoryLinearSystem<U>& other)
    : TrajectoryLinearSystem<T>(*other.A_, *other.B_, *other.C_, *other.D_,
                                other.time_period()) {
  this->ConfigureDefaultAndRandomStateFrom(other);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::TrajectoryLinearSystem)
