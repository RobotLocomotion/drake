#pragma once

#include <gtest/gtest.h>
#include "ignition/math/Vector3.hh"

namespace drake {
namespace maliput {
namespace rndf {
namespace test {

// Compares equality within @p tolerance deviation of two
// ignition::math::Vector3d objects.
// @param v1 An ignition::math::Vector3d object to compare.
// @param v2 An ignition::math::Vector3d object to compare.
// @param tolerance An allowable absolute deviation for each
// ignition::math::Vector3d's coordinate.
// @return ::testing::AssertionFailure() When ignition::math::Vector3d objects
// are different.
// @return ::testing::AssertionSuccess() When ignition::math::Vector3d objects
// are within the @p tolerance deviation.
::testing::AssertionResult IsIgnitionVector3dClose(
    const ignition::math::Vector3d& v1, const ignition::math::Vector3d& v2,
    double tolerance);

}  // namespace test
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
