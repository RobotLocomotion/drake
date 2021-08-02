#include "drake/multibody/parsing/detail_ignition.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;

Vector3d ToVector3(const ignition::math::Vector3d& vector) {
  return Vector3d(vector.X(), vector.Y(), vector.Z());
}

RigidTransformd ToRigidTransform(const ignition::math::Pose3d& pose) {
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return RigidTransformd(rotation, ToVector3(pose.Pos()));;
}

ignition::math::Pose3d ToIgnitionPose3d(const RigidTransformd& pose) {
  const auto& quat = pose.rotation().ToQuaternion();
  return ignition::math::Pose3d(
      ignition::math::Vector3d(pose.translation().x(), pose.translation().y(),
                               pose.translation().z()),
      ignition::math::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z()));
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
