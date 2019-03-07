#pragma once

#include <ignition/math/Pose3.hh>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace detail {

/// Helper function to express an ignition::math::Vector3d instance as
/// a Vector3d instance.
Eigen::Vector3d ToVector3(const ignition::math::Vector3d& vector);

/// Helper function to express an ignition::math::Pose3d instance as
/// an Isometry3d instance.
Eigen::Isometry3d ToIsometry3(const ignition::math::Pose3d& pose);

}  // namespace detail
}  // namespace multibody
}  // namespace drake
