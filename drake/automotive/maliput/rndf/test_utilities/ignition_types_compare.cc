#include "drake/automotive/maliput/rndf/test_utilities/ignition_types_compare.h"

#include <cmath>
#include <string>

namespace drake {
namespace maliput {
namespace rndf {
namespace test {

::testing::AssertionResult IsIgnitionVector3dClose(
    const ignition::math::Vector3d& v1, const ignition::math::Vector3d& v2,
    double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(v1.X() - v2.X());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "ignition::math::Vector3d are different at X coordinate. "
                   + "v1.X(): " + std::to_string(v1.X()) + " vs. v2.X(): "
                   + std::to_string(v2.X()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(v1.Y() - v2.Y());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "ignition::math::Vector3d are different at Y coordinate. "
                   + "v1.Y(): " + std::to_string(v1.Y()) + " vs. v2.Y(): "
                   + std::to_string(v2.Y()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(v1.Z() - v2.Z());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "ignition::math::Vector3d are different at Z coordinate. "
                   + "v1.Z(): " + std::to_string(v1.Z()) + " vs. v2.Z(): "
                   + std::to_string(v2.Z()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }

  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "v1 =\n"
                                       << v1
                                       << "\nis approximately equal to v2 =\n"
                                       << v2
                                       << ", tolerance = " << tolerance;
}

}  // namespace test
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
