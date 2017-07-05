#pragma once

#include <cmath>

#include <gtest/gtest.h>
#include "ignition/math/Vector3.hh"

namespace drake {
namespace maliput {
namespace rndf {

// Compares equality within @p tolerance deviation of two
// ignition::math::Vector3d objects.
// @param v1 An ignition::math::Vector3d object to compare.
// @param pos2 An ignition::math::Vector3d object to compare.
// @param tolerance An allowable deviation for each ignition::math::Vector3d's
// coordinate.
// @return ::testing::AssertionFailure() When ignition::math::Vector3d objects
// are different.
// @return ::testing::AssertionSuccess() When ignition::math::Vector3d objects
// are equal or within the @p tolerance deviation.
::testing::AssertionResult CompareIgnitionVector3d(
    const ignition::math::Vector3d& v1, const ignition::math::Vector3d& v2,
    double tolerance = 0.0) {
  double delta = std::abs(v1.X() - v2.X());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "ignition::math::Vector3d are different at X coordinate. "
           << "v1.X(): " << v1.X() << " vs. "
           << "v2.X(): " << v2.X() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(v1.Y() - v2.Y());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "GeoPositions are different at Y coordinate. "
           << "v1.Y(): " << v1.Y() << " vs. "
           << "v2.Y(): " << v2.Y() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(v1.Z() - v2.Z());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "GeoPositions are different at Z coordinate. "
           << "v1.Z(): " << v1.Z() << " vs. "
           << "v2.Z(): " << v2.Z() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  return ::testing::AssertionSuccess() << "v1 =\n"
                                       << v1
                                       << "\nis approximately equal to v2 =\n"
                                       << v2;
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
