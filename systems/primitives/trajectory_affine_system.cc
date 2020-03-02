#include "drake/systems/primitives/trajectory_affine_system.h"

namespace drake {
namespace systems {

template <typename T>
TrajectoryAffineSystem<T>::TrajectoryAffineSystem(
    const trajectories::Trajectory<double>& A,
    const trajectories::Trajectory<double>& B,
    const trajectories::Trajectory<double>& f0,
    const trajectories::Trajectory<double>& C,
    const trajectories::Trajectory<double>& D,
    const trajectories::Trajectory<double>& y0,
    double time_period)
    : TimeVaryingAffineSystem<T>(
          SystemTypeTag<TrajectoryAffineSystem>{},
          A.rows(), B.cols(), C.rows(), time_period),
      A_(A.Clone()),
      B_(B.Clone()),
      f0_(f0.Clone()),
      C_(C.Clone()),
      D_(D.Clone()),
      y0_(y0.Clone()) {}

template <typename T>
template <typename U>
TrajectoryAffineSystem<T>::TrajectoryAffineSystem(
    const TrajectoryAffineSystem<U>& other)
    : TrajectoryAffineSystem<T>(*other.A_, *other.B_, *other.f0_, *other.C_,
                                *other.D_, *other.y0_, other.time_period()) {
  this->ConfigureDefaultAndRandomStateFrom(other);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::TrajectoryAffineSystem)
