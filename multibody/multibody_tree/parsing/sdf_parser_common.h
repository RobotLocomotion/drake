#pragma once

#include <ignition/math/Pose3.hh>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Default value of the Coulomb's law coefficients of friction for when they
/// are not specified in the SDF file.
// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// http://drake.mit.edu/styleguide/cppguide.html#Static_and_Global_Variables
// For this reason, we create and return an instance of CoulombFriction
// instead of using a static variable.
inline multibody_plant::CoulombFriction<double> default_friction() {
  return multibody_plant::CoulombFriction<double>(1.0, 1.0);
}

namespace detail {

/// Helper function to express an ignition::math::Vector3d instance as
/// a Vector3d instance.
Eigen::Vector3d ToVector3(const ignition::math::Vector3d& vector);

/// Helper function to express an ignition::math::Pose3d instance as
/// an Isometry3d instance.
Eigen::Isometry3d ToIsometry3(const ignition::math::Pose3d& pose);

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
