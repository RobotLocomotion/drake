#pragma once

#include <cmath>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace maliput {
namespace api {

// Compares equality within @p tolerance deviation of two GeoPositions objects.
// @param pos1 A GeoPosition object to compare.
// @param pos2 A GeoPosition object to compare.
// @param tolerance An allowable deviation for each GeoPosition's coordinate.
// @return ::testing::AssertionFailure() When GeoPosition objects are different.
// @return ::testing::AssertionSuccess() When GeoPosition objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult CompareGeoPositions(const GeoPosition& pos1,
                                               const GeoPosition& pos2,
                                               double tolerance = 0.0) {
  double delta = std::abs(pos1.x() - pos2.x());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "GeoPositions are different at x coordinate. "
           << "pos1.x(): " << pos1.x() << " vs. "
           << "pos2.x(): " << pos2.x() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(pos1.y() - pos2.y());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "GeoPositions are different at y coordinate. "
           << "pos1.y(): " << pos1.y() << " vs. "
           << "pos2.y(): " << pos2.y() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(pos1.z() - pos2.z());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "GeoPositions are different at z coordinate. "
           << "pos1.z(): " << pos1.z() << " vs. "
           << "pos2.z(): " << pos2.z() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  return ::testing::AssertionSuccess() << "pos1 =\n"
                                       << pos1
                                       << "\nis approximately equal to pos2 =\n"
                                       << pos2;
}

// Compares equality within @p tolerance deviation of two LanePosition objects.
// @param pos1 A LanePosition object to compare.
// @param pos2 A LanePosition object to compare.
// @param tolerance An allowable deviation for each LanePosition's coordinate.
// @return ::testing::AssertionFailure() When LanePosition objects are
// different.
// @return ::testing::AssertionSuccess() When LanePosition objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult CompareLanePositions(const LanePosition& pos1,
                                                const LanePosition& pos2,
                                                double tolerance = 0.0) {
  double delta = std::abs(pos1.s() - pos2.s());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "LanePositions are different at s coordinate. "
           << "pos1.s(): " << pos1.s() << " vs. "
           << "pos2.s(): " << pos2.s() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(pos1.r() - pos2.r());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "LanePositions are different at r coordinate. "
           << "pos1.r(): " << pos1.r() << " vs. "
           << "pos2.r(): " << pos2.r() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(pos1.h() - pos2.h());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "LanePositions are different at h coordinate. "
           << "pos1.h(): " << pos1.h() << " vs. "
           << "pos2.h(): " << pos2.h() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  return ::testing::AssertionSuccess() << "pos1 =\n"
                                       << pos1
                                       << "\nis approximately equal to pos2 =\n"
                                       << pos2;
}

// Compares equality within @p tolerance deviation of two Rotation objects.
// Comparison will evaluate the inner Rotation's Quaternion.
// @param rot1 A Rotation object to compare.
// @param rot2 A Rotation object to compare.
// @param tolerance An allowable deviation for each Rotation's coordinate.
// @return ::testing::AssertionFailure() When Rotation objects are different.
// @return ::testing::AssertionSuccess() When Rotation objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult CompareRotations(const Rotation& rot1,
                                            const Rotation& rot2,
                                            double tolerance = 0.0) {
  const Quaternion<double>& quat1 = rot1.quat();
  const Quaternion<double>& quat2 = rot2.quat();

  double delta = std::abs(quat1.x() - quat2.x());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "Rotation's quaternions are different at x coordinate. "
           << "rot1.quat().x(): " << quat1.x() << " vs. "
           << "rot2.quat().x(): " << quat2.x() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(quat1.y() - quat2.y());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "Rotation's quaternions are different at y coordinate. "
           << "rot1.quat().y(): " << quat1.y() << " vs. "
           << "rot2.quat().y(): " << quat2.y() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(quat1.z() - quat2.z());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "Rotation's quaternions are different at z coordinate. "
           << "rot1.quat().z(): " << quat1.z() << " vs. "
           << "rot2.quat().z(): " << quat2.z() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(quat1.w() - quat2.w());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "Rotation's quaternions are different at w coordinate. "
           << "rot1.quat().w(): " << quat1.w() << " vs. "
           << "rot2.quat().w(): " << quat2.w() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  return ::testing::AssertionSuccess() << "rot1 =\n"
                                       << rot1
                                       << "\nis approximately equal to rot2 =\n"
                                       << rot2;
}

// Compares equality within @p tolerance deviation of two RBounds objects.
// @param rbounds1 A RBounds object to compare.
// @param rbounds2 A RBounds object to compare.
// @param tolerance An allowable deviation for each IsoLaneVelocity's
// coordinate.
// @return ::testing::AssertionFailure() When RBounds objects are different.
// @return ::testing::AssertionSuccess() When RBounds objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult CompareRBounds(const RBounds& rbounds1,
                                          const RBounds& rbounds2,
                                          double tolerance = 0.0) {
  double delta = std::abs(rbounds1.r_min - rbounds2.r_min);
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "RBounds are different at r_min. "
           << "rbounds1.r_min: " << rbounds1.r_min << " vs. "
           << "rbounds2.r_min: " << rbounds2.r_min << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(rbounds1.r_max - rbounds2.r_max);
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "RBounds are different at r_max. "
           << "rbounds1.r_max: " << rbounds1.r_max << " vs. "
           << "rbounds2.r_max: " << rbounds2.r_max << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  return ::testing::AssertionSuccess()
         << "rbounds1 =\n"
         << "(" << rbounds1.r_min << ", " << rbounds1.r_max << ")"
         << "\nis approximately equal to "
         << " rbounds2 =\n"
         << "(" << rbounds2.r_min << ", " << rbounds2.r_max << ")";
}

// Compares equality within @p tolerance deviation of two HBounds objects.
// @param hbounds1 A HBounds object to compare.
// @param hbounds1 A HBounds object to compare.
// @param tolerance An allowable deviation for each IsoLaneVelocity's
// coordinate.
// @return ::testing::AssertionFailure() When HBounds objects are different.
// @return ::testing::AssertionSuccess() When HBounds objects are equal or
// within the @p tolerance deviation.
::testing::AssertionResult CompareHBounds(const HBounds& hbounds1,
                                          const HBounds& hbounds2,
                                          double tolerance = 0.0) {
  double delta = std::abs(hbounds1.min() - hbounds2.min());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "HBounds are different at min. "
           << "rbounds1.min(): " << hbounds1.min() << " vs. "
           << "rbounds2.min(): " << hbounds2.min() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  delta = std::abs(hbounds1.max() - hbounds2.max());
  if (delta > tolerance) {
    return ::testing::AssertionFailure()
           << "HBounds are different at max. "
           << "rbounds1.max(): " << hbounds1.max() << " vs. "
           << "rbounds2.max(): " << hbounds2.max() << ", diff = " << delta
           << ", tolerance = " << tolerance;
  }
  return ::testing::AssertionSuccess()
         << "hbounds1 =\n"
         << "(" << hbounds1.min() << ", " << hbounds1.max() << ")"
         << "\nis approximately equal to "
         << " hbounds2 =\n"
         << "(" << hbounds2.min() << ", " << hbounds2.max() << ")";
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
