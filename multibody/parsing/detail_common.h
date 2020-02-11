#pragma once

#include <optional>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace internal {

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

/// Populates an instance of geometry::ProximityProperties from a reading
/// interface in a URDF/SDF agnostic manner. This unifies the URDF and SDF
/// parsing logic and eliminates code redundancy. The individual URDF and SDF
/// parsers have the following responsibilities (based on the simple fact that
/// the two parsers use different mechanisms to extract data from the file):
///
///   1. Determine if the `<drake:rigid_hydroelastic>` tag is present.
///   2. Determine if the `<drake:soft_hydroelastic>` tag is present.
///   3. Create a function that will extract an *optional* double-valued scalar
///      from a <drake:some_property> child tag of the
///      <drake:proximity_properties> tag.
///
/// This function does limited semantic parsing. For example, if for a
/// particular application, a set of coordinated properties are required, this
/// parsing method does *not* validate that set. It's sole purpose is to
/// parse supported parameters and store them in expected values in the
/// properties. Downstream consumers of those properties are responsible for
/// confirming that all required properties are present and well formed.
///
/// @param read_double  The function for extracting double values for specific
///                     named tags.
/// @param is_rigid     True if the caller detected the presence of the
///                     <drake:rigid_hydroelastic> tag.
/// @param is_soft      True if the caller detected the presence of the
///                     <drake:soft_hydroelastic> tag.
/// @return All proximity properties discovered via the `read_double` function.
/// @pre At most one of `is_rigid` and `is_soft` is true.
geometry::ProximityProperties ParseProximityProperties(
    const std::function<std::optional<double>(const char*)>& read_double,
    bool is_rigid, bool is_soft);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
