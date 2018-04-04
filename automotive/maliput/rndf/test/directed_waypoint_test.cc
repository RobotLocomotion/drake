#include "drake/automotive/maliput/rndf/directed_waypoint.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <ignition/math/Vector3.hh>
#include <ignition/rndf/UniqueId.hh>

#include "drake/automotive/maliput/rndf/test_utilities/ignition_types_compare.h"

namespace drake {
namespace maliput {
namespace rndf {
namespace {

// Checks the DirectedWaypoint's default constructor and the getters.
GTEST_TEST(RNDFBuilderTest, DirectedWaypointDefaultConstructorTest) {
  const double kVeryExactTolerance = 1e-12;
  DirectedWaypoint dw;
  const ignition::rndf::UniqueId default_id;
  EXPECT_EQ(dw.id().X(), default_id.X());
  EXPECT_EQ(dw.id().Y(), default_id.Y());
  EXPECT_EQ(dw.id().Z(), default_id.Z());
  const ignition::math::Vector3d zero_vector = ignition::math::Vector3d::Zero;
  EXPECT_TRUE(test::IsIgnitionVector3dClose(dw.position(), zero_vector,
                                            kVeryExactTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(dw.tangent(), zero_vector,
                                            kVeryExactTolerance));
  EXPECT_FALSE(dw.is_entry());
  EXPECT_FALSE(dw.is_exit());
}

// Checks the DirectedWaypoint's setters.
GTEST_TEST(RNDFBuilderTest, DirectedWaypointSetterTest) {
  const double kVeryExactTolerance = 1e-12;
  DirectedWaypoint dw;
  const ignition::rndf::UniqueId id(1, 2, 3);
  dw.set_id(id);
  EXPECT_EQ(dw.id().X(), id.X());
  EXPECT_EQ(dw.id().Y(), id.Y());
  EXPECT_EQ(dw.id().Z(), id.Z());
  const ignition::math::Vector3d position(1.0, 2.0, 3.0);
  dw.set_position(position);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(dw.position(), position,
                                            kVeryExactTolerance));
  const ignition::math::Vector3d tangent(4.0, 5.0, 6.0);
  dw.set_tangent(tangent);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(dw.tangent(), tangent,
                                            kVeryExactTolerance));
  const bool is_entry{true};
  dw.set_is_entry(is_entry);
  EXPECT_EQ(dw.is_entry(), is_entry);
  const bool is_exit{true};
  dw.set_is_exit(is_exit);
  EXPECT_EQ(dw.is_exit(), is_exit);
}

// Checks the DirectedWaypoint's parameter constructor.
GTEST_TEST(RNDFBuilderTest, DirectedWaypointParameterConstructorTest) {
  const double kVeryExactTolerance = 1e-12;
  const ignition::rndf::UniqueId id(1, 2, 3);
  const ignition::math::Vector3d position(1.0, 2.0, 3.0);
  const ignition::math::Vector3d tangent(4.0, 5.0, 6.0);
  const bool kIsEntry{true};
  const bool kIsExit{false};
  DirectedWaypoint dw(id, position, tangent, kIsEntry, kIsExit);
  EXPECT_EQ(dw.id().X(), id.X());
  EXPECT_EQ(dw.id().Y(), id.Y());
  EXPECT_EQ(dw.id().Z(), id.Z());
  EXPECT_TRUE(test::IsIgnitionVector3dClose(dw.position(), position,
                                            kVeryExactTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(dw.tangent(), tangent,
                                            kVeryExactTolerance));
  EXPECT_EQ(dw.is_entry(), kIsEntry);
  EXPECT_EQ(dw.is_exit(), kIsExit);
}

// Checks the computation of the BoundingBox of different DirectedWaypoints.
GTEST_TEST(RNDFBuilderTest, DirectedWaypointBoundingBoxTest) {
  const double kZeroTolerance = 0.0;
  std::vector<DirectedWaypoint> waypoints;
  std::pair<ignition::math::Vector3d, ignition::math::Vector3d> extents;
  // Checks the empty-vector case.
  extents = DirectedWaypoint::CalculateBoundingBox(waypoints);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      extents.first, ignition::math::Vector3d::Zero, kZeroTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      extents.second, ignition::math::Vector3d::Zero, kZeroTolerance));
  // Checks a rotated square case.
  waypoints.push_back(DirectedWaypoint());
  waypoints.push_back(DirectedWaypoint());
  waypoints.push_back(DirectedWaypoint());
  waypoints.push_back(DirectedWaypoint());
  waypoints.push_back(DirectedWaypoint());
  const double kHalfDiagonal = 5.0;
  waypoints[0].set_position(ignition::math::Vector3d(-kHalfDiagonal, 0.0, 0.0));
  waypoints[1].set_position(ignition::math::Vector3d(0.0, kHalfDiagonal, 0.0));
  waypoints[2].set_position(ignition::math::Vector3d(kHalfDiagonal, 0.0, 0.0));
  waypoints[3].set_position(ignition::math::Vector3d(0.0, -kHalfDiagonal, 0.0));
  waypoints[4].set_position(
      ignition::math::Vector3d(kHalfDiagonal / 2.0, -kHalfDiagonal / 2.0, 0.0));
  extents = DirectedWaypoint::CalculateBoundingBox(waypoints);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      extents.first,
      ignition::math::Vector3d(-kHalfDiagonal, -kHalfDiagonal, 0.0),
      kZeroTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      extents.second,
      ignition::math::Vector3d(kHalfDiagonal, kHalfDiagonal, 0.0),
      kZeroTolerance));
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
