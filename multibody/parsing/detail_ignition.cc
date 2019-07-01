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

}  // namespace internal
}  // namespace multibody
}  // namespace drake
