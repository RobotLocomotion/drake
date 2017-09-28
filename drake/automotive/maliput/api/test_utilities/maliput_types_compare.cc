#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"

#include <cmath>
#include <string>

namespace drake {
namespace maliput {
namespace api {
namespace test {

::testing::AssertionResult IsGeoPositionClose(const GeoPosition& pos1,
                                              const GeoPosition& pos2,
                                              double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(pos1.x() - pos2.x());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "GeoPositions are different at x coordinate. "
                   + "pos1.x(): " + std::to_string(pos1.x()) + " vs. "
                   + "pos2.x(): " + std::to_string(pos2.x()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.y() - pos2.y());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "GeoPositions are different at y coordinate. "
                   + "pos1.y(): " + std::to_string(pos1.y()) + " vs. "
                   + "pos2.y(): " + std::to_string(pos2.y()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.z() - pos2.z());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "GeoPositions are different at z coordinate. "
                   + "pos1.z(): " + std::to_string(pos1.z()) + " vs. "
                   + "pos2.z(): " + std::to_string(pos2.z()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "pos1 =\n"
                                       << pos1
                                       << "\nis approximately equal to pos2 =\n"
                                       << pos2
                                       << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsLanePositionClose(const LanePosition& pos1,
                                               const LanePosition& pos2,
                                               double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(pos1.s() - pos2.s());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "LanePositions are different at s coordinate. "
                   + "pos1.s(): " + std::to_string(pos1.s()) + " vs. "
                   + "pos2.s(): " + std::to_string(pos2.s()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.r() - pos2.r());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "LanePositions are different at r coordinate. "
                   + "pos1.r(): " + std::to_string(pos1.r()) + " vs. "
                   + "pos2.r(): " + std::to_string(pos2.r()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.h() - pos2.h());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "LanePositions are different at h coordinate. "
                   + "pos1.h(): " + std::to_string(pos1.h()) + " vs. "
                   + "pos2.h(): " + std::to_string(pos2.h()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "pos1 =\n"
                                       << pos1
                                       << "\nis approximately equal to pos2 =\n"
                                       << pos2
                                       << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsRotationClose(const Rotation& rot1,
                                           const Rotation& rot2,
                                           double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(rot1.roll() - rot2.roll());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "Rotations are different at roll angle. "
                   + "rot1.roll(): " + std::to_string(rot1.roll()) + " vs. "
                   + "rot2.roll(): " + std::to_string(rot2.roll()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(rot1.pitch() - rot2.pitch());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "Rotations are different at pitch angle. "
                   + "rot1.pitch(): " + std::to_string(rot1.pitch()) + " vs. "
                   + "rot2.pitch(): " + std::to_string(rot1.pitch())
                   + ", diff = " + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(rot1.yaw() - rot2.yaw());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "Rotations are different at yaw angle. "
                   + "rot1.yaw(): " + std::to_string(rot2.yaw()) + " vs. "
                   + "rot2.yaw(): " + std::to_string(rot2.yaw()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "rot1 =\n"
                                       << rot1
                                       << "\nis approximately equal to rot2 =\n"
                                       << rot2
                                       << " within " << tolerance
                                       << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsRBoundsClose(const RBounds& rbounds1,
                                          const RBounds& rbounds2,
                                          double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(rbounds1.min() - rbounds2.min());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                    + "RBounds are different at r_min. "
                    + "rbounds1.r_min: " + std::to_string(rbounds1.min())
                    + " vs. rbounds2.r_min: " + std::to_string(rbounds2.min())
                    + ", diff = " + std::to_string(delta) + ", tolerance = "
                    + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(rbounds1.max() - rbounds2.max());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                    + "RBounds are different at r_max. "
                    + "rbounds1.r_max: " + std::to_string(rbounds1.max())
                    + " vs. rbounds2.r_max: " + std::to_string(rbounds2.max())
                    + ", diff = " + std::to_string(delta) + ", tolerance = "
                    + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess()
         << "rbounds1 =\n"
         << "(" << rbounds1.min() << ", " << rbounds1.max() << ")"
         << "\nis approximately equal to "
         << " rbounds2 =\n"
         << "(" << rbounds2.min() << ", " << rbounds2.max() << ")"
         << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsHBoundsClose(const HBounds& hbounds1,
                                          const HBounds& hbounds2,
                                          double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(hbounds1.min() - hbounds2.min());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "HBounds are different at min. rbounds1.min(): "
                   + std::to_string(hbounds1.min()) + " vs. rbounds2.min(): "
                   + std::to_string(hbounds2.min()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(hbounds1.max() - hbounds2.max());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message
                   + "HBounds are different at max. rbounds1.max(): "
                   + std::to_string(hbounds1.max()) + " vs. rbounds2.max(): "
                   + std::to_string(hbounds2.max()) + ", diff = "
                   + std::to_string(delta) + ", tolerance = "
                   + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess()
         << "hbounds1 =\n"
         << "(" << hbounds1.min() << ", " << hbounds1.max() << ")"
         << "\nis approximately equal to "
         << " hbounds2 =\n"
         << "(" << hbounds2.min() << ", " << hbounds2.max() << ")"
         << ", tolerance = " << tolerance;
}

}  // namespace test
}  // namespace api
}  // namespace maliput
}  // namespace drake
