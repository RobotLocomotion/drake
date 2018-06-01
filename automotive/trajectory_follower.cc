#include "drake/automotive/trajectory_follower.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace automotive {

using multibody::SpatialVelocity;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

template <typename T>
TrajectoryFollower<T>::TrajectoryFollower(const Trajectory& trajectory,
                                          double sampling_time_sec)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::TrajectoryFollower>{}),
      trajectory_(trajectory) {
  this->DeclarePeriodicUnrestrictedUpdate(sampling_time_sec, 0.);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcStateOutput);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcPoseOutput);
  this->DeclareVectorOutputPort(&TrajectoryFollower::CalcVelocityOutput);
}

template <typename T>
void TrajectoryFollower<T>::CalcStateOutput(
    const systems::Context<T>& context,
    SimpleCarState<T>* output_vector) const {
  const PoseVelocity values = GetValues(context);
  output_vector->set_x(T{values.pose3().x()});
  output_vector->set_y(T{values.pose3().y()});
  output_vector->set_heading(T{values.pose3().z()});
  output_vector->set_velocity(T{values.speed()});
}

template <typename T>
void TrajectoryFollower<T>::CalcPoseOutput(const systems::Context<T>& context,
                                        PoseVector<T>* pose) const {
  const PoseVelocity values = GetValues(context);
  pose->set_translation(Eigen::Translation<T, 3>{values.translation()});
  const Eigen::Quaternion<double>& q =
      math::RotationMatrix<double>(values.rotation()).ToQuaternion();
  pose->set_rotation(Eigen::Quaternion<T>{q});
}

template <typename T>
void TrajectoryFollower<T>::CalcVelocityOutput(
    const systems::Context<T>& context,
    FrameVelocity<T>* velocity) const {
  const PoseVelocity values = GetValues(context);
  const Eigen::Vector3d& v = values.velocity().translational();
  const Eigen::Vector3d& w = values.velocity().rotational();
  velocity->set_velocity(SpatialVelocity<T>{Vector3<T>{w}, Vector3<T>{v}});
}

template <typename T>
PoseVelocity TrajectoryFollower<T>::GetValues(
    const systems::Context<T>& context) const {
  return trajectory_.value(ExtractDoubleOrThrow(context.get_time()));
}

}  // namespace automotive
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::TrajectoryFollower)
