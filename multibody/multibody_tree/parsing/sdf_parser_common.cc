#include "drake/multibody/multibody_tree/parsing/sdf_parser_common.h"

#include <sdf/sdf.hh>

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

using Eigen::Isometry3d;
using Eigen::Vector3d;

// Helper function to express an ignition::math::Vector3d instance as
// a Vector3d instance.
Vector3d ToVector3(const ignition::math::Vector3d& vector) {
  return Vector3d(vector.X(), vector.Y(), vector.Z());
}

// Helper function to express an ignition::math::Pose3d instance as
// an Isometry3d instance.
Isometry3d ToIsometry3(const ignition::math::Pose3d& pose) {
  const Isometry3d::TranslationType translation(ToVector3(pose.Pos()));
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return translation * rotation;
}

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
