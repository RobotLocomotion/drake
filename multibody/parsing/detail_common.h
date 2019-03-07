#pragma once

#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace detail {

// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// http://drake.mit.edu/styleguide/cppguide.html#Static_and_Global_Variables
// For this reason, we create and return an instance of CoulombFriction
// instead of using a static variable.
/// Default value of the Coulomb's law coefficients of friction for when they
/// are not specified in the URDF/SDF file.
inline CoulombFriction<double> default_friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

}  // namespace detail
}  // namespace multibody
}  // namespace drake
