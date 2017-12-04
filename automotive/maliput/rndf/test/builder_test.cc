#include "drake/automotive/maliput/rndf/builder.h"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include <ignition/math/Vector3.hh>
#include <ignition/rndf/UniqueId.hh>

#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {
namespace {

const double kLaneWidth = 5.0;
const double kLinearTolerance = 1e-2;
const double kAngularTolerance = 1e-2 * M_PI;

int FindJunction(const api::RoadGeometry& road_geometry,
                 const std::string& junction_name) {
  for (int i = 0; i < road_geometry.num_junctions(); i++) {
    if (road_geometry.junction(i)->id().string() == junction_name) {
      return i;
    }
  }
  return -1;
}

//   * 1.1.1
//    v
//      v
//        * 1.1.2
//      v
//    v
//   * 1.1.3
//    v
//      v
//        * 1.1.4
//
// For reference:
//   - 'v' represents lane's direction.
//   - '*' represents a lane's waypoint.
GTEST_TEST(RNDFBuilder, ZigZagLane) {
  auto builder = std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);

  std::vector<DirectedWaypoint> waypoints(4, DirectedWaypoint());
  waypoints[0].set_id(ignition::rndf::UniqueId(1, 1, 1));
  waypoints[0].set_position(ignition::math::Vector3d(0., 0.0, 0.0));
  waypoints[1].set_id(ignition::rndf::UniqueId(1, 1, 2));
  waypoints[1].set_position(ignition::math::Vector3d(10.0, -10.0, 0.0));
  waypoints[2].set_id(ignition::rndf::UniqueId(1, 1, 3));
  waypoints[2].set_position(ignition::math::Vector3d(0.0, -20.0, 0.0));
  waypoints[3].set_id(ignition::rndf::UniqueId(1, 1, 4));
  waypoints[3].set_position(ignition::math::Vector3d(10.0, -30.0, 0.0));
  const Connection connection("1", waypoints, kLaneWidth, false);

  std::vector<Connection> connected_lanes = {connection};

  const auto bounding_box =
      std::make_pair(ignition::math::Vector3d(0., -30., 0.),
                     ignition::math::Vector3d(10., 0., 0.));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  const auto road_geometry = builder->Build(api::RoadGeometryId{"ZigZagLane"});
  ASSERT_TRUE(road_geometry != nullptr);

  // Checks junctions, segments and lanes.
  ASSERT_EQ(road_geometry->num_junctions(), 3);
  EXPECT_EQ(road_geometry->junction(0)->id().string(), "j:1-0-0");
  ASSERT_EQ(road_geometry->junction(0)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().string(), "s:1-0-0");
  ASSERT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().string(),
            "l:1.1.1-1.1.2");

  EXPECT_EQ(road_geometry->junction(1)->id().string(), "j:1-0-1");
  ASSERT_EQ(road_geometry->junction(1)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().string(), "s:1-0-1");
  ASSERT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->id().string(),
            "l:1.1.2-1.1.3");

  EXPECT_EQ(road_geometry->junction(2)->id().string(), "j:1-0-2");
  ASSERT_EQ(road_geometry->junction(2)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().string(), "s:1-0-2");
  ASSERT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->id().string(),
            "l:1.1.3-1.1.4");

  // Checks branch points.
  ASSERT_EQ(road_geometry->num_branch_points(), 4);
  EXPECT_EQ(road_geometry->branch_point(0)->id().string(), "bp:0");
  EXPECT_EQ(road_geometry->branch_point(0)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(0)->GetBSide()->size(), 0);

  EXPECT_EQ(road_geometry->branch_point(1)->id().string(), "bp:1");
  EXPECT_EQ(road_geometry->branch_point(1)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(1)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(2)->id().string(), "bp:2");
  EXPECT_EQ(road_geometry->branch_point(2)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(2)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(3)->id().string(), "bp:3");
  EXPECT_EQ(road_geometry->branch_point(3)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(3)->GetBSide()->size(), 0);

  // Checks branch point assigment regarding the lanes.
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->GetBranchPoint(
                api::LaneEnd::kStart),
            road_geometry->branch_point(0));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->GetBranchPoint(
                api::LaneEnd::kFinish),
            road_geometry->branch_point(1));

  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->GetBranchPoint(
                api::LaneEnd::kStart),
            road_geometry->branch_point(1));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->GetBranchPoint(
                api::LaneEnd::kFinish),
            road_geometry->branch_point(2));

  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->GetBranchPoint(
                api::LaneEnd::kStart),
            road_geometry->branch_point(2));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->GetBranchPoint(
                api::LaneEnd::kFinish),
            road_geometry->branch_point(3));
}

// * 1.2.11  * 1.1.1         1.1.9 *  1.2.1 *
// ^         v                     ^        v
// ^         v                     ^        v
// * 1.2.10  * 1.1.2         1.1.8 *  1.2.2 *
// ^         v                     ^        v
// ^         v                     ^        v
// * 1.2.9   * 1.1.3         1.1.7 *  1.2.3 *
//  ^         v                   ^        v
//   ^         v                 ^        v
//    * 1.2.8   * 1.1.4   1.1.6 *  1.2.4 *
//     ^         v             ^        v
//      ^          < <  *  < <         v
//       ^            1.1.5           v
//         * 1.2.7             1.2.5 *
//           > > > > >  *  > > > > >
//                    1.2.6
//
// For reference:
//   -'^', 'v', '<' and '>' represent lane's direction.
//   - '*' represents a lane's waypoint.
GTEST_TEST(RNDFBuilder, UShapedLane) {
  auto builder = std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);

  std::vector<Connection> connected_lanes;
  {
    std::vector<DirectedWaypoint> waypoints(9, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 1, 1));
    waypoints[0].set_position(ignition::math::Vector3d(-20.0, 50.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 1, 2));
    waypoints[1].set_position(ignition::math::Vector3d(-20.0, 40.0, 0.0));
    waypoints[2].set_id(ignition::rndf::UniqueId(1, 1, 3));
    waypoints[2].set_position(ignition::math::Vector3d(-20.0, 30.0, 0.0));
    waypoints[3].set_id(ignition::rndf::UniqueId(1, 1, 4));
    waypoints[3].set_position(ignition::math::Vector3d(-10.0, 20.0, 0.0));
    waypoints[4].set_id(ignition::rndf::UniqueId(1, 1, 5));
    waypoints[4].set_position(ignition::math::Vector3d(0.0, 10.0, 0.0));
    waypoints[5].set_id(ignition::rndf::UniqueId(1, 1, 6));
    waypoints[5].set_position(ignition::math::Vector3d(10.0, 20.0, 0.0));
    waypoints[6].set_id(ignition::rndf::UniqueId(1, 1, 7));
    waypoints[6].set_position(ignition::math::Vector3d(20.0, 30.0, 0.0));
    waypoints[7].set_id(ignition::rndf::UniqueId(1, 1, 8));
    waypoints[7].set_position(ignition::math::Vector3d(20.0, 40.0, 0.0));
    waypoints[8].set_id(ignition::rndf::UniqueId(1, 1, 9));
    waypoints[8].set_position(ignition::math::Vector3d(20.0, 50.0, 0.0));
    connected_lanes.push_back(Connection("1", waypoints, kLaneWidth, false));
  }

  {
    std::vector<DirectedWaypoint> waypoints(11, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 2, 1));
    waypoints[0].set_position(ignition::math::Vector3d(40.0, 50.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 2, 2));
    waypoints[1].set_position(ignition::math::Vector3d(40.0, 40.0, 0.0));
    waypoints[2].set_id(ignition::rndf::UniqueId(1, 2, 3));
    waypoints[2].set_position(ignition::math::Vector3d(40.0, 30.0, 0.0));
    waypoints[3].set_id(ignition::rndf::UniqueId(1, 2, 4));
    waypoints[3].set_position(ignition::math::Vector3d(30.0, 20.0, 0.0));
    waypoints[4].set_id(ignition::rndf::UniqueId(1, 2, 5));
    waypoints[4].set_position(ignition::math::Vector3d(20, 5.0, 0.0));
    waypoints[5].set_id(ignition::rndf::UniqueId(1, 2, 6));
    waypoints[5].set_position(ignition::math::Vector3d(0.0, 0.0, 0.0));
    waypoints[6].set_id(ignition::rndf::UniqueId(1, 2, 7));
    waypoints[6].set_position(ignition::math::Vector3d(-20.0, 5.0, 0.0));
    waypoints[7].set_id(ignition::rndf::UniqueId(1, 2, 8));
    waypoints[7].set_position(ignition::math::Vector3d(-30.0, 20.0, 0.0));
    waypoints[8].set_id(ignition::rndf::UniqueId(1, 2, 9));
    waypoints[8].set_position(ignition::math::Vector3d(-40.0, 30.0, 0.0));
    waypoints[9].set_id(ignition::rndf::UniqueId(1, 2, 10));
    waypoints[9].set_position(ignition::math::Vector3d(-40.0, 40.0, 0.0));
    waypoints[10].set_id(ignition::rndf::UniqueId(1, 2, 11));
    waypoints[10].set_position(ignition::math::Vector3d(-40.0, 50.0, 0.0));
    connected_lanes.push_back(Connection("2", waypoints, kLaneWidth, false));
  }

  const auto bounding_box =
      std::make_pair(ignition::math::Vector3d(-40.0, 0.0, 0.0),
                     ignition::math::Vector3d(40., 50.0, 0.0));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  const auto road_geometry = builder->Build(api::RoadGeometryId{"UShapedLane"});
  ASSERT_TRUE(road_geometry != nullptr);

  // Checks junction, segments and lanes. Lane naming implies direction
  // so checking for correct naming implies proper direction inference.
  const std::vector<std::tuple<std::string, std::string, std::string>>
      name_truth_table{
          // Tuple elements are, in order: junction name, segment name and lane
          // name.
          std::make_tuple("j:1-0-0", "s:1-0-0", "l:1.1.1-1.1.2"),
          std::make_tuple("j:1-0-1", "s:1-0-1", "l:1.1.2-1.1.3"),
          std::make_tuple("j:1-0-2", "s:1-0-2", "l:1.1.3-1.1.4"),
          std::make_tuple("j:1-0-3", "s:1-0-3", "l:1.1.4-1.1.5"),
          std::make_tuple("j:1-0-4", "s:1-0-4", "l:1.1.5-1.1.6"),
          std::make_tuple("j:1-0-5", "s:1-0-5", "l:1.1.6-1.1.7"),
          std::make_tuple("j:1-0-6", "s:1-0-6", "l:1.1.7-1.1.8"),
          std::make_tuple("j:1-0-7", "s:1-0-7", "l:1.1.8-1.1.9"),
          std::make_tuple("j:1-1-0", "s:1-1-0", "l:1.2.1-1.2.2"),
          std::make_tuple("j:1-1-1", "s:1-1-1", "l:1.2.2-1.2.3"),
          std::make_tuple("j:1-1-2", "s:1-1-2", "l:1.2.3-1.2.4"),
          std::make_tuple("j:1-1-3", "s:1-1-3", "l:1.2.4-1.2.5"),
          std::make_tuple("j:1-1-4", "s:1-1-4", "l:1.2.5-1.2.6"),
          std::make_tuple("j:1-1-5", "s:1-1-5", "l:1.2.6-1.2.7"),
          std::make_tuple("j:1-1-6", "s:1-1-6", "l:1.2.7-1.2.8"),
          std::make_tuple("j:1-1-7", "s:1-1-7", "l:1.2.8-1.2.9"),
          std::make_tuple("j:1-1-8", "s:1-1-8", "l:1.2.9-1.2.10"),
          std::make_tuple("j:1-1-9", "s:1-1-9", "l:1.2.10-1.2.11")};

  for (const auto& names : name_truth_table) {
    std::string junction_name, segment_name, lane_name;
    std::tie(junction_name, segment_name, lane_name) = names;
    const int junction_id = FindJunction(*road_geometry, junction_name);
    ASSERT_NE(junction_id, -1);
    const api::Junction* junction = road_geometry->junction(junction_id);
    ASSERT_TRUE(junction != nullptr);
    ASSERT_EQ(junction->num_segments(), 1);
    const api::Segment* segment = junction->segment(0);
    ASSERT_TRUE(segment != nullptr);
    EXPECT_EQ(segment->id().string(), segment_name);
    ASSERT_EQ(segment->num_lanes(), 1);
    const api::Lane* lane = segment->lane(0);
    ASSERT_TRUE(lane != nullptr);
    EXPECT_EQ(lane->id().string(), lane_name);
  }
}

//          1.1.1      1.1.2       1.1.3
//          * > > > > > * > > > > > *
//  1.2.1   1.2.2  1.2.3    1.2.4
//  * > > > * > > > * > > > > *
//              1.3.2                    1.3.1
//              * < < < < < < < < < < < < *
//
// For reference:
//   -'<' and '>' represent lane's direction.
//   - '*' represents a lane's waypoint.
GTEST_TEST(RNDFBuilder, MultilaneLane) {
  const api::RBounds single_lane_bounds(-kLaneWidth / 2.0, kLaneWidth / 2.0);
  const api::RBounds two_lane_bounds_left(-kLaneWidth / 2.0,
                                          kLaneWidth / 2.0 + 10.0);
  const api::RBounds two_lane_bounds_right(-kLaneWidth / 2.0 - 10.0,
                                           kLaneWidth / 2.0);

  auto builder = std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);
  std::vector<Connection> connected_lanes;

  {
    std::vector<DirectedWaypoint> waypoints(4, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 2, 1));
    waypoints[0].set_position(ignition::math::Vector3d(0., 0.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 2, 2));
    waypoints[1].set_position(ignition::math::Vector3d(10.0, 0.0, 0.0));
    waypoints[2].set_id(ignition::rndf::UniqueId(1, 2, 3));
    waypoints[2].set_position(ignition::math::Vector3d(15.0, 0.0, 0.0));
    waypoints[3].set_id(ignition::rndf::UniqueId(1, 2, 4));
    waypoints[3].set_position(ignition::math::Vector3d(25.0, 0.0, 0.0));
    const Connection connection("1", waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }

  {
    std::vector<DirectedWaypoint> waypoints(3, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 1, 1));
    waypoints[0].set_position(ignition::math::Vector3d(10., 10.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 1, 2));
    waypoints[1].set_position(ignition::math::Vector3d(20.0, 10.0, 0.0));
    waypoints[2].set_id(ignition::rndf::UniqueId(1, 1, 3));
    waypoints[2].set_position(ignition::math::Vector3d(30.0, 10.0, 0.0));
    const Connection connection("1", waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }

  {
    std::vector<DirectedWaypoint> waypoints(2, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 3, 1));
    waypoints[0].set_position(ignition::math::Vector3d(40., -10.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 3, 2));
    waypoints[1].set_position(ignition::math::Vector3d(5.0, -10.0, 0.0));
    const Connection connection("1", waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }

  const auto bounding_box =
      std::make_pair(ignition::math::Vector3d(0., -10., 0.),
                     ignition::math::Vector3d(40., 10., 0.));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  const auto road_geometry =
      builder->Build(api::RoadGeometryId{"MultilaneLane"});
  ASSERT_TRUE(road_geometry != nullptr);

  // Check road creation, naming, which lane is at both sides and bounds.
  ASSERT_EQ(road_geometry->num_junctions(), 6);
  EXPECT_EQ(road_geometry->junction(0)->id().string(), "j:1-0-0");
  ASSERT_EQ(road_geometry->junction(0)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().string(), "s:1-0-0");
  ASSERT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().string(),
            "l:1.2.1-1.2.2");
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->to_left(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->to_right(),
            nullptr);
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(0)->segment(0)->lane(0)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(0)->segment(0)->lane(0)->driveable_bounds(0),
      single_lane_bounds, kLinearTolerance));

  EXPECT_EQ(road_geometry->junction(1)->id().string(), "j:1-0-1");
  ASSERT_EQ(road_geometry->junction(1)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().string(), "s:1-0-1");
  ASSERT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->id().string(),
            "l:1.2.2-1.2.3");
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(1)->id().string(),
            "l:1.1.1-1.1.5");
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->to_left(),
            road_geometry->junction(1)->segment(0)->lane(1));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->to_right(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(1)->to_left(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(1)->to_right(),
            road_geometry->junction(1)->segment(0)->lane(0));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(1)->segment(0)->lane(0)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(1)->segment(0)->lane(1)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(1)->segment(0)->lane(0)->driveable_bounds(0),
      two_lane_bounds_left, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(1)->segment(0)->lane(1)->driveable_bounds(0),
      two_lane_bounds_right, kLinearTolerance));

  EXPECT_EQ(road_geometry->junction(2)->id().string(), "j:1-0-2");
  ASSERT_EQ(road_geometry->junction(2)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().string(), "s:1-0-2");
  ASSERT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->id().string(),
            "l:1.2.3-1.2.5");
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(1)->id().string(),
            "l:1.1.5-1.1.2");
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->to_left(),
            road_geometry->junction(2)->segment(0)->lane(1));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->to_right(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(1)->to_left(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(1)->to_right(),
            road_geometry->junction(2)->segment(0)->lane(0));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(2)->segment(0)->lane(0)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(2)->segment(0)->lane(1)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(2)->segment(0)->lane(0)->driveable_bounds(0),
      two_lane_bounds_left, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(2)->segment(0)->lane(1)->driveable_bounds(0),
      two_lane_bounds_right, kLinearTolerance));

  EXPECT_EQ(road_geometry->junction(3)->id().string(), "j:1-0-3");
  ASSERT_EQ(road_geometry->junction(3)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->id().string(), "s:1-0-3");
  ASSERT_EQ(road_geometry->junction(3)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(0)->id().string(),
            "l:1.2.5-1.2.4");
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(1)->id().string(),
            "l:1.1.2-1.1.6");
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(0)->to_left(),
            road_geometry->junction(3)->segment(0)->lane(1));
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(0)->to_right(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(1)->to_left(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(1)->to_right(),
            road_geometry->junction(3)->segment(0)->lane(0));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(3)->segment(0)->lane(0)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(3)->segment(0)->lane(1)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(3)->segment(0)->lane(0)->driveable_bounds(0),
      two_lane_bounds_left, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(3)->segment(0)->lane(1)->driveable_bounds(0),
      two_lane_bounds_right, kLinearTolerance));

  EXPECT_EQ(road_geometry->junction(4)->id().string(), "j:1-0-4");
  ASSERT_EQ(road_geometry->junction(4)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->id().string(), "s:1-0-4");
  ASSERT_EQ(road_geometry->junction(4)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->lane(0)->id().string(),
            "l:1.1.6-1.1.3");
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->lane(0)->to_left(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->lane(0)->to_right(),
            nullptr);
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(4)->segment(0)->lane(0)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(4)->segment(0)->lane(0)->driveable_bounds(0),
      single_lane_bounds, kLinearTolerance));

  EXPECT_EQ(road_geometry->junction(5)->id().string(), "j:1-1-0");
  ASSERT_EQ(road_geometry->junction(5)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->id().string(), "s:1-1-0");
  ASSERT_EQ(road_geometry->junction(5)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->lane(0)->id().string(),
            "l:1.3.1-1.3.2");
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->lane(0)->to_left(),
            nullptr);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->lane(0)->to_right(),
            nullptr);
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(5)->segment(0)->lane(0)->lane_bounds(0),
      single_lane_bounds, kLinearTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      road_geometry->junction(5)->segment(0)->lane(0)->driveable_bounds(0),
      single_lane_bounds, kLinearTolerance));
}

//               1.1.1      1.2.3
//                     *   *
//                     v   ^
//               1.1.2 v   ^
//    2.2.2    2.2.1  /*   *
//     * < < < < < < * v  /^ 1.2.2
//    2.1.1    2.1.2   v / ^           2.1.3
//     * > > > > > > * > > > > > > > *
//                   | v   ^
//               1.1.3 *   ^
//                     v   ^
//                     v   ^
//               1.1.4 *   * 1.2.1
// For reference:
//   -'^', 'v', '<' and '>' represent lane's direction.
//   -'|' and '/' represent crossing intersections.
//   - '*' represents a lane's waypoint.
GTEST_TEST(RNDFBuilder, MultilaneLaneCross) {
  auto builder = std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);
  const auto bounding_box =
      std::make_pair(ignition::math::Vector3d(0., 0., 0.),
                     ignition::math::Vector3d(40., 50., 0.));
  builder->SetBoundingBox(bounding_box);

  std::vector<Connection> connected_lanes;
  {
    std::vector<DirectedWaypoint> waypoints(4, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 1, 1));
    waypoints[0].set_position(ignition::math::Vector3d(20., 50.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 1, 2));
    waypoints[1].set_position(ignition::math::Vector3d(20.0, 40.0, 0.0));
    waypoints[2].set_id(ignition::rndf::UniqueId(1, 1, 3));
    waypoints[2].set_position(ignition::math::Vector3d(20.0, 10.0, 0.0));
    waypoints[3].set_id(ignition::rndf::UniqueId(1, 1, 4));
    waypoints[3].set_position(ignition::math::Vector3d(20.0, 0.0, 0.0));
    const Connection connection("1", waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }
  {
    std::vector<DirectedWaypoint> waypoints(3, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 2, 1));
    waypoints[0].set_position(ignition::math::Vector3d(30., 0.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 2, 2));
    waypoints[1].set_position(ignition::math::Vector3d(30.0, 30.0, 0.0));
    waypoints[2].set_id(ignition::rndf::UniqueId(1, 2, 3));
    waypoints[2].set_position(ignition::math::Vector3d(30.0, 50.0, 0.0));
    const Connection connection("1", waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }
  builder->CreateSegmentConnections(1, &connected_lanes);

  connected_lanes.clear();
  {
    std::vector<DirectedWaypoint> waypoints(3, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(2, 1, 1));
    waypoints[0].set_position(ignition::math::Vector3d(0., 20.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(2, 1, 2));
    waypoints[1].set_position(ignition::math::Vector3d(10.0, 20.0, 0.0));
    waypoints[2].set_id(ignition::rndf::UniqueId(2, 1, 3));
    waypoints[2].set_position(ignition::math::Vector3d(40.0, 20.0, 0.0));
    const Connection connection("2", waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }
  {
    std::vector<DirectedWaypoint> waypoints(2, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(2, 2, 1));
    waypoints[0].set_position(ignition::math::Vector3d(10., 30.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(2, 2, 2));
    waypoints[1].set_position(ignition::math::Vector3d(0.0, 30.0, 0.0));
    const Connection connection("2", waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }
  builder->CreateSegmentConnections(2, &connected_lanes);

  builder->CreateConnection(kLaneWidth, ignition::rndf::UniqueId(1, 1, 2),
                            ignition::rndf::UniqueId(2, 2, 1));
  builder->CreateConnection(kLaneWidth, ignition::rndf::UniqueId(2, 1, 2),
                            ignition::rndf::UniqueId(1, 2, 2));
  builder->CreateConnection(kLaneWidth, ignition::rndf::UniqueId(2, 1, 2),
                            ignition::rndf::UniqueId(1, 1, 3));

  const auto road_geometry =
      builder->Build(api::RoadGeometryId{"MultilaneLaneCross"});
  ASSERT_TRUE(road_geometry != nullptr);

  // Checks lane creation, naming, and bound coordinates.
  ASSERT_EQ(road_geometry->num_junctions(), 11);

  const std::vector<std::tuple<std::string, std::string, std::string,
                               api::GeoPosition, api::GeoPosition>>
      lane_truth_table{
          // Tuple elements are, in order: junction name, segment name, lane
          // name, lane start api::GeoPosition and lane end api::GeoPosition.
          std::make_tuple("j:1-0-0", "s:1-0-0", "l:1.1.1-1.1.2",
                          api::GeoPosition(20.0, 50.0, 0.0),
                          api::GeoPosition(20.0, 40.0, 0.0)),
          std::make_tuple("j:1-0-1", "s:1-0-1", "l:1.1.2-1.1.3",
                          api::GeoPosition(20.0, 40.0, 0.0),
                          api::GeoPosition(20.0, 10.0, 0.0)),
          std::make_tuple("j:1-0-2", "s:1-0-2", "l:1.1.3-1.1.4",
                          api::GeoPosition(20.0, 10.0, 0.0),
                          api::GeoPosition(20.0, 0.0, 0.0)),
          std::make_tuple("j:1-1-0", "s:1-1-0", "l:1.2.1-1.2.2",
                          api::GeoPosition(30.0, 0.0, 0.0),
                          api::GeoPosition(30.0, 30.0, 0.0)),
          std::make_tuple("j:1-1-1", "s:1-1-1", "l:1.2.2-1.2.3",
                          api::GeoPosition(30.0, 30.0, 0.0),
                          api::GeoPosition(30.0, 50.0, 0.0)),
          std::make_tuple("j:2-0-0", "s:2-0-0", "l:2.1.1-2.1.2",
                          api::GeoPosition(0.0, 20.0, 0.0),
                          api::GeoPosition(10.0, 20.0, 0.0)),
          std::make_tuple("j:2-0-1", "s:2-0-1", "l:2.1.2-2.1.3",
                          api::GeoPosition(10.0, 20.0, 0.0),
                          api::GeoPosition(40.0, 20.0, 0.0)),
          std::make_tuple("j:2-1-0", "s:2-1-0", "l:2.2.1-2.2.2",
                          api::GeoPosition(10.0, 30.0, 0.0),
                          api::GeoPosition(0.0, 30.0, 0.0)),
          std::make_tuple("j:1.1.2-2.2.1", "s:1.1.2-2.2.1", "l:1.1.2-2.2.1",
                          api::GeoPosition(20.0, 40.0, 0.0),
                          api::GeoPosition(10.0, 30.0, 0.0)),
          std::make_tuple("j:2.1.2-1.2.2", "s:2.1.2-1.2.2", "l:2.1.2-1.2.2",
                          api::GeoPosition(10.0, 20.0, 0.0),
                          api::GeoPosition(30.0, 30.0, 0.0)),
          std::make_tuple("j:2.1.2-1.1.3", "s:2.1.2-1.1.3", "l:2.1.2-1.1.3",
                          api::GeoPosition(10.0, 20.0, 0.0),
                          api::GeoPosition(20.0, 10.0, 0.0))};

  for (const auto& values : lane_truth_table) {
    std::string junction_name, segment_name, lane_name;
    api::GeoPosition start_position, end_position;
    std::tie(junction_name, segment_name, lane_name, start_position,
             end_position) = values;

    const int junction_id = FindJunction(*road_geometry, junction_name);
    ASSERT_NE(junction_id, -1);
    ASSERT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().string(),
              segment_name);
    ASSERT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);
    const api::Lane* lane =
        road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().string(), lane_name);
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)), start_position,
        kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        end_position, kLinearTolerance));
  }

  // Checks branch point creation.
  ASSERT_EQ(road_geometry->num_branch_points(), 12);

  const std::vector<std::tuple<std::string, int, int, api::LaneEnd::Which>>
      branch_point_truth_table{
          // Tuple items are, in order: junction name, branch point A side lane
          // count, branch point B side lane count and api::LaneEnd::Which type.
          std::make_tuple("j:1-0-0", 1, 2, api::LaneEnd::kFinish),
          std::make_tuple("j:1-0-1", 2, 1, api::LaneEnd::kFinish),
          std::make_tuple("j:1-1-1", 2, 1, api::LaneEnd::kStart)};

  for (const auto& values : branch_point_truth_table) {
    std::string junction_name;
    int a_side_count = 0, b_side_count = 0;
    api::LaneEnd::Which lane_end = api::LaneEnd::kStart;
    std::tie(junction_name, a_side_count, b_side_count, lane_end) = values;

    const int junction_id = FindJunction(*road_geometry, junction_name);
    ASSERT_NE(junction_id, -1);
    const api::BranchPoint* branch_point = road_geometry->junction(junction_id)
                                               ->segment(0)
                                               ->lane(0)
                                               ->GetBranchPoint(lane_end);
    ASSERT_TRUE(branch_point != nullptr);
    EXPECT_EQ(branch_point->GetASide()->size(), a_side_count);
    EXPECT_EQ(branch_point->GetBSide()->size(), b_side_count);
  }
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
