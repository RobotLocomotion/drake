#include "drake/multibody/parsing/detail_ignition.h"

namespace drake {
namespace multibody {
namespace internal {

using Eigen::Vector3d;
using math::RigidTransformd;

Vector3d ToVector3(const gz::math::Vector3d& vector) {
  return Vector3d(vector.X(), vector.Y(), vector.Z());
}

RigidTransformd ToRigidTransform(const gz::math::Pose3d& pose) {
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return RigidTransformd(rotation, ToVector3(pose.Pos()));
}

gz::math::Pose3d ToIgnitionPose3d(const RigidTransformd& pose) {
  const auto& quat = pose.rotation().ToQuaternion();
  return gz::math::Pose3d(
      gz::math::Vector3d(pose.translation().x(), pose.translation().y(),
                         pose.translation().z()),
      gz::math::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z()));
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
