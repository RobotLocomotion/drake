#include "drake/automotive/maliput/rndf/connection.h"

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

// Checks Connection's constructor constraints that may throw exceptions.
GTEST_TEST(RNDFBuilderTest, ConnectionConstructorTest) {
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
  std::vector<DirectedWaypoint> directed_waypoints(2, DirectedWaypoint());
  directed_waypoints[0].set_id(ignition::rndf::UniqueId(1, 1, 1));
  directed_waypoints[0].set_position(ignition::math::Vector3d(0.0, 0.0, 0.0));
  directed_waypoints[1].set_id(ignition::rndf::UniqueId(1, 1, 2));
  directed_waypoints[1].set_position(ignition::math::Vector3d(10.0, 0.0, 0.0));
  EXPECT_THROW(Connection("", directed_waypoints, -kWidth, kInverseDirection),
               std::runtime_error);
  EXPECT_NO_THROW(Connection("l:1_1_1-1_1_2", directed_waypoints, kWidth,
                             kInverseDirection));
}

// Checks Connection's setters and getters and the constraints that may throw
// exceptions.
GTEST_TEST(RNDFBuilderTest, ConnectionSetterAndGetterTest) {
  const double kWidth = 5.0;
  const bool kInverseDirection = false;
  // Creates a connection.
  std::vector<DirectedWaypoint> directed_waypoints(2, DirectedWaypoint());
  const ignition::rndf::UniqueId first_id(1, 1, 1);
  directed_waypoints[0].set_id(first_id);
  const ignition::math::Vector3d first_position(0.0, 0.0, 0.0);
  directed_waypoints[0].set_position(first_position);
  const ignition::rndf::UniqueId second_id(1, 1, 2);
  directed_waypoints[1].set_id(second_id);
  const ignition::math::Vector3d second_position(10.0, 0.0, 0.0);
  directed_waypoints[1].set_position(second_position);
  Connection connection("l:1_1_1-1_1_2", directed_waypoints, kWidth,
                        kInverseDirection);
  // Check start() and end() methods.
  EXPECT_EQ(connection.start().id().X(), first_id.X());
  EXPECT_EQ(connection.start().id().Y(), first_id.Y());
  EXPECT_EQ(connection.start().id().Z(), first_id.Z());
  EXPECT_EQ(connection.end().id().X(), second_id.X());
  EXPECT_EQ(connection.end().id().Y(), second_id.Y());
  EXPECT_EQ(connection.end().id().Z(), second_id.Z());
  // Check the inverse_direction related methods.
  EXPECT_EQ(connection.inverse_direction(), kInverseDirection);
  connection.set_inverse_direction(!kInverseDirection);
  EXPECT_EQ(connection.inverse_direction(), !kInverseDirection);
  // Checks the width's setter constraint.
  EXPECT_THROW(connection.set_width(0.0), std::runtime_error);
  EXPECT_THROW(connection.set_width(-kWidth), std::runtime_error);
  EXPECT_NO_THROW(connection.set_width(2.0 * kWidth));
  EXPECT_EQ(connection.width(), (2.0 * kWidth));
}

// Checks different methods to modify the Connection's DirectedWaypoint vector.
GTEST_TEST(RNDFBuilderTest, ConnectionWaypointTest) {
  const double kWidth = 5.0;
  const bool kInverseDirection = false;
  // Creates a connection.
  std::vector<DirectedWaypoint> directed_waypoints(2, DirectedWaypoint());
  const ignition::rndf::UniqueId first_id(1, 1, 1);
  directed_waypoints[0].set_id(first_id);
  const ignition::math::Vector3d first_position(0.0, 0.0, 0.0);
  directed_waypoints[0].set_position(first_position);
  const ignition::rndf::UniqueId second_id(1, 1, 2);
  directed_waypoints[1].set_id(second_id);
  const ignition::math::Vector3d second_position(10.0, 0.0, 0.0);
  directed_waypoints[1].set_position(second_position);
  Connection connection("l:1_1_1-1_1_2", directed_waypoints, kWidth,
                        kInverseDirection);
  // Checks that at least two valid DirectedWaypoints are set.
  EXPECT_THROW(connection.set_waypoints(
                   std::vector<DirectedWaypoint>(3, DirectedWaypoint())),
               std::runtime_error);
  // Checks invalid indexes in AddWaypoint().
  EXPECT_THROW(connection.AddWaypoint(DirectedWaypoint(), -1),
               std::runtime_error);
  EXPECT_THROW(connection.AddWaypoint(DirectedWaypoint(), 5),
               std::runtime_error);
  // Checks that the waypoint is added at the beginning.
  const ignition::rndf::UniqueId id_at_the_beginning(1, 1, 3);
  const DirectedWaypoint directed_waypoint_at_the_beginning(
      id_at_the_beginning, ignition::math::Vector3d(),
      ignition::math::Vector3d(), false, false);
  connection.AddWaypoint(directed_waypoint_at_the_beginning, 0);
  EXPECT_EQ(connection.waypoints().size(), 3);
  EXPECT_EQ(connection.waypoints()[0].id().X(), id_at_the_beginning.X());
  EXPECT_EQ(connection.waypoints()[0].id().Y(), id_at_the_beginning.Y());
  EXPECT_EQ(connection.waypoints()[0].id().Z(), id_at_the_beginning.Z());
  // Checks that the waypoint is added at the middle.
  const ignition::rndf::UniqueId id_at_the_middle(1, 1, 5);
  const DirectedWaypoint directed_waypoint_at_the_middle(
      id_at_the_middle, ignition::math::Vector3d(), ignition::math::Vector3d(),
      false, false);
  connection.AddWaypoint(directed_waypoint_at_the_middle, 1);
  EXPECT_EQ(connection.waypoints().size(), 4);
  EXPECT_EQ(connection.waypoints()[1].id().X(), id_at_the_middle.X());
  EXPECT_EQ(connection.waypoints()[1].id().Y(), id_at_the_middle.Y());
  EXPECT_EQ(connection.waypoints()[1].id().Z(), id_at_the_middle.Z());
  // Checks that the waypoint is added at the end.
  const ignition::rndf::UniqueId id_at_the_end(1, 1, 6);
  const DirectedWaypoint directed_waypoint_at_the_end(
      id_at_the_end, ignition::math::Vector3d(), ignition::math::Vector3d(),
      false, false);
  connection.AddWaypoint(directed_waypoint_at_the_end,
                         connection.waypoints().size());
  EXPECT_EQ(connection.waypoints().size(), 5);
  EXPECT_EQ(connection.waypoints()[4].id().X(), id_at_the_end.X());
  EXPECT_EQ(connection.waypoints()[4].id().Y(), id_at_the_end.Y());
  EXPECT_EQ(connection.waypoints()[4].id().Z(), id_at_the_end.Z());
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
