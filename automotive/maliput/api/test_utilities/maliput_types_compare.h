#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"

namespace drake {
namespace maliput {
namespace api {
namespace test {

// Compares equality within @p tolerance deviation of two GeoPosition objects.
// @param pos1 A GeoPosition object to compare.
// @param pos2 A GeoPosition object to compare.
// @param tolerance An allowable absolute deviation for each GeoPosition's
// coordinate.
// @return ::testing::AssertionFailure() When GeoPosition objects are different.
// @return ::testing::AssertionSuccess() When GeoPosition objects are within
// the @p tolerance deviation.
::testing::AssertionResult IsGeoPositionClose(const GeoPosition& pos1,
                                              const GeoPosition& pos2,
                                              double tolerance);

// Compares equality within @p tolerance deviation of two LanePosition objects.
// @param pos1 A LanePosition object to compare.
// @param pos2 A LanePosition object to compare.
// @param tolerance An allowable absolute deviation for each LanePosition's
// coordinate.
// @return ::testing::AssertionFailure() When LanePosition objects are
// different.
// @return ::testing::AssertionSuccess() When LanePosition objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsLanePositionClose(const LanePosition& pos1,
                                               const LanePosition& pos2,
                                               double tolerance);

// Compares equality within @p tolerance deviation of two Rotation objects.
// Comparison will evaluate the inner Rotation's Euler angles.
// @param rot1 A Rotation object to compare.
// @param rot2 A Rotation object to compare.
// @param tolerance An allowable absolute deviation for each Rotation's
// coordinate.
// @return ::testing::AssertionFailure() When Rotation objects are different.
// @return ::testing::AssertionSuccess() When Rotation objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsRotationClose(const Rotation& rot1,
                                           const Rotation& rot2,
                                           double tolerance);

// Compares equality within @p tolerance deviation of two RBounds objects.
// @param rbounds1 A RBounds object to compare.
// @param rbounds2 A RBounds object to compare.
// @param tolerance An allowable absolute deviation for each RBounds's instance
// value.
// @return ::testing::AssertionFailure() When RBounds objects are different.
// @return ::testing::AssertionSuccess() When RBounds objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsRBoundsClose(const RBounds& rbounds1,
                                          const RBounds& rbounds2,
                                          double tolerance);

// Compares equality within @p tolerance deviation of two HBounds objects.
// @param hbounds1 A HBounds object to compare.
// @param hbounds1 A HBounds object to compare.
// @param tolerance An allowable absolute deviation for each HBounds's instance
// value.
// @return ::testing::AssertionFailure() When HBounds objects are different.
// @return ::testing::AssertionSuccess() When HBounds objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult IsHBoundsClose(const HBounds& hbounds1,
                                          const HBounds& hbounds2,
                                          double tolerance);

}  // namespace test
}  // namespace api
}  // namespace maliput
}  // namespace drake
