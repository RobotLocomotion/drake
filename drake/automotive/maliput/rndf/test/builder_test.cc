#include "drake/automotive/maliput/rndf/builder.h"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include <ignition/math/Vector3.hh>

#include <ignition/rndf/UniqueId.hh>

#include "drake/automotive/maliput/rndf/test/ignition_types_compare.h"

namespace drake {
namespace maliput {
namespace rndf {

// Checks constructors, setters and getters of DirectedWaypoints.
GTEST_TEST(RNDFBuilderTest, DirectedWaypointTest) {
  const double kVeryExact = 1e-12;
  DirectedWaypoint dw;
  // Checks default values and getters.
  EXPECT_EQ(dw.id().X(), -1);
  EXPECT_EQ(dw.id().Y(), -1);
  EXPECT_EQ(dw.id().Z(), -1);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      dw.position(), ignition::math::Vector3d::Zero, kVeryExact));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      dw.tangent(), ignition::math::Vector3d::Zero, kVeryExact));
  EXPECT_FALSE(dw.is_entry());
  EXPECT_FALSE(dw.is_exit());
  // Checks the setters.
  dw.set_id(ignition::rndf::UniqueId(1, 2, 3));
  dw.set_position(ignition::math::Vector3d(1.0, 2.0, 3.0));
  dw.set_tangent(ignition::math::Vector3d(4.0, 5.0, 6.0));
  dw.set_is_entry(true);
  dw.set_is_exit(false);
  EXPECT_EQ(dw.id().X(), 1);
  EXPECT_EQ(dw.id().Y(), 2);
  EXPECT_EQ(dw.id().Z(), 3);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      dw.position(), ignition::math::Vector3d(1.0, 2.0, 3.0), kVeryExact));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      dw.tangent(), ignition::math::Vector3d(4.0, 5.0, 6.0), kVeryExact));
  EXPECT_TRUE(dw.is_entry());
  EXPECT_FALSE(dw.is_exit());
  // Checks the parameter constructor.
  DirectedWaypoint dw2(ignition::rndf::UniqueId(1, 2, 3),
                       ignition::math::Vector3d(1.0, 2.0, 3.0),
                       ignition::math::Vector3d(4.0, 5.0, 6.0), true, false);
  EXPECT_EQ(dw2.id().X(), 1);
  EXPECT_EQ(dw2.id().Y(), 2);
  EXPECT_EQ(dw2.id().Z(), 3);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      dw2.position(), ignition::math::Vector3d(1.0, 2.0, 3.0), kVeryExact));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      dw2.tangent(), ignition::math::Vector3d(4.0, 5.0, 6.0), kVeryExact));
  EXPECT_TRUE(dw2.is_entry());
  EXPECT_FALSE(dw2.is_exit());
}

// Checks the computation of the BoundingBox of different DirectedWaypoints.
GTEST_TEST(RNDFBuilderTest, DirectedWaypointBoundingBoxTest) {
  const double kVeryExact = 1e-12;
  std::vector<DirectedWaypoint> waypoints;
  std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> extents;
  // Checks the no items in vector case.
  extents = DirectedWaypoint::CalculateBoundingBox(waypoints);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      std::get<0>(extents), ignition::math::Vector3d::Zero, kVeryExact));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      std::get<1>(extents), ignition::math::Vector3d::Zero, kVeryExact));
  // Checks a rotated square case.
  waypoints.push_back(DirectedWaypoint());
  waypoints.push_back(DirectedWaypoint());
  waypoints.push_back(DirectedWaypoint());
  waypoints.push_back(DirectedWaypoint());
  waypoints[0].set_position(ignition::math::Vector3d(-5.0, 0.0, 0.0));
  waypoints[1].set_position(ignition::math::Vector3d(0.0, 5.0, 0.0));
  waypoints[2].set_position(ignition::math::Vector3d(5.0, 0.0, 0.0));
  waypoints[3].set_position(ignition::math::Vector3d(0.0, -5.0, 0.0));
  extents = DirectedWaypoint::CalculateBoundingBox(waypoints);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      std::get<0>(extents), ignition::math::Vector3d(-5.0, -5.0, 0.0),
      kVeryExact));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      std::get<1>(extents), ignition::math::Vector3d(5.0, 5.0, 0.0),
      kVeryExact));
}

// Checks methods of the Connection that are sensitive to throw exceptions.
GTEST_TEST(RNDFBuilderTest, ConnectionTest) {
  const double kWidth = 5.0;
  const bool kInverseDirection = false;
  // Checks DirectedWaypoint vector constraints.
  EXPECT_THROW(Connection("", std::vector<DirectedWaypoint>(), kWidth,
                          kInverseDirection),
               std::runtime_error);
  EXPECT_THROW(
      Connection("", std::vector<DirectedWaypoint>(3, DirectedWaypoint()),
                 kWidth, kInverseDirection),
      std::runtime_error);
  // Checks width constraints.
  std::vector<DirectedWaypoint> dwp(2, DirectedWaypoint());
  dwp[0].set_id(ignition::rndf::UniqueId(1, 1, 1));
  dwp[0].set_position(ignition::math::Vector3d(0.0, 0.0, 0.0));
  dwp[1].set_id(ignition::rndf::UniqueId(1, 1, 2));
  dwp[1].set_position(ignition::math::Vector3d(10.0, 0.0, 0.0));
  EXPECT_THROW(
      Connection("", std::vector<DirectedWaypoint>(3, DirectedWaypoint()),
                 -kWidth, kInverseDirection),
      std::runtime_error);
  Connection connection("l:1_1_1-1_1_2", dwp, kWidth, kInverseDirection);
  // Checks the constraint of set_waypoint.
  EXPECT_THROW(connection.set_waypoints(
                   std::vector<DirectedWaypoint>(3, DirectedWaypoint())),
               std::runtime_error);
  // Checks the constraints of AddWaypoint.
  EXPECT_THROW(connection.AddWaypoint(DirectedWaypoint(), -1),
               std::runtime_error);
  EXPECT_THROW(connection.AddWaypoint(DirectedWaypoint(), 5),
               std::runtime_error);
  // Checks that the waypoint is added at the beginning.
  connection.AddWaypoint(DirectedWaypoint(), 0);
  EXPECT_EQ(connection.waypoints().size(), 3);
  EXPECT_EQ(connection.waypoints()[0].id().X(), -1);
  EXPECT_EQ(connection.waypoints()[0].id().Y(), -1);
  EXPECT_EQ(connection.waypoints()[0].id().Z(), -1);
  // Checks that the waypoint is added at the middle.
  DirectedWaypoint dw;
  dw.set_id(ignition::rndf::UniqueId(1, 1, 3));
  connection.AddWaypoint(dw, 1);
  EXPECT_EQ(connection.waypoints().size(), 4);
  EXPECT_EQ(connection.waypoints()[1].id().X(), 1);
  EXPECT_EQ(connection.waypoints()[1].id().Y(), 1);
  EXPECT_EQ(connection.waypoints()[1].id().Z(), 3);
  // Checks that the waypoint is added at the end.
  dw.set_id(ignition::rndf::UniqueId(1, 1, 4));
  connection.AddWaypoint(DirectedWaypoint(), connection.waypoints().size());
  EXPECT_EQ(connection.waypoints().size(), 5);
  EXPECT_EQ(connection.waypoints()[4].id().X(), -1);
  EXPECT_EQ(connection.waypoints()[4].id().Y(), -1);
  EXPECT_EQ(connection.waypoints()[4].id().Z(), -1);
  // Checks the width's setter constraint.
  EXPECT_THROW(connection.set_width(0.0), std::runtime_error);
  EXPECT_THROW(connection.set_width(-10.0), std::runtime_error);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
