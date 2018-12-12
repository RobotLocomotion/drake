#include "drake/multibody/parsing/detail_ignition.h"

namespace drake {
namespace multibody {
namespace detail {

using Eigen::Isometry3d;
using Eigen::Vector3d;

Vector3d ToVector3(const ignition::math::Vector3d& vector) {
  return Vector3d(vector.X(), vector.Y(), vector.Z());
}

Isometry3d ToIsometry3(const ignition::math::Pose3d& pose) {
  const Isometry3d::TranslationType translation(ToVector3(pose.Pos()));
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return translation * rotation;
}

}  // namespace detail
}  // namespace multibody
}  // namespace drake
