#include "drake/automotive/maliput/rndf/builder.h"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include <ignition/math/Vector3.hh>
#include <ignition/rndf/UniqueId.hh>

#include "drake/automotive/maliput/api/test/maliput_types_compare.h"
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
    if (road_geometry.junction(i)->id().id == junction_name) {
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
GTEST_TEST(RNDFBuilder, ZigZagLane) {
  std::unique_ptr<Builder> builder =
      std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);

  std::vector<DirectedWaypoint> waypoints(4, DirectedWaypoint());
  waypoints[0].set_id(ignition::rndf::UniqueId(1, 1, 1));
  waypoints[0].set_position(ignition::math::Vector3d(0., 0.0, 0.0));
  waypoints[1].set_id(ignition::rndf::UniqueId(1, 1, 2));
  waypoints[1].set_position(ignition::math::Vector3d(10.0, -10.0, 0.0));
  waypoints[2].set_id(ignition::rndf::UniqueId(1, 1, 3));
  waypoints[2].set_position(ignition::math::Vector3d(0.0, -20.0, 0.0));
  waypoints[3].set_id(ignition::rndf::UniqueId(1, 1, 4));
  waypoints[3].set_position(ignition::math::Vector3d(10.0, -30.0, 0.0));
  Connection connection(std::to_string(1), waypoints, kLaneWidth, false);

  std::vector<Connection> connected_lanes = {connection};

  auto bounding_box =
      std::make_tuple<ignition::math::Vector3d, ignition::math::Vector3d>(
          ignition::math::Vector3d(0., -30., 0.),
          ignition::math::Vector3d(10., 0., 0.));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"ZigZagLane"});
  EXPECT_NE(road_geometry, nullptr);

  // Check the junctions, segments and lanes
  EXPECT_EQ(road_geometry->num_junctions(), 3);
  EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id,
            std::string("s:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id,
            std::string("l:1.1.1-1.1.2"));

  EXPECT_EQ(road_geometry->junction(1)->id().id, std::string("j:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id,
            std::string("s:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->id().id,
            std::string("l:1.1.2-1.1.3"));

  EXPECT_EQ(road_geometry->junction(2)->id().id, std::string("j:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id,
            std::string("s:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->id().id,
            std::string("l:1.1.3-1.1.4"));

  // Check the branchpoints
  EXPECT_EQ(road_geometry->num_branch_points(), 4);
  EXPECT_EQ(road_geometry->branch_point(0)->id().id,
            std::string("bp:") + std::to_string(0));
  EXPECT_EQ(road_geometry->branch_point(0)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(0)->GetBSide()->size(), 0);

  EXPECT_EQ(road_geometry->branch_point(1)->id().id,
            std::string("bp:") + std::to_string(1));
  EXPECT_EQ(road_geometry->branch_point(1)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(1)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(2)->id().id,
            std::string("bp:") + std::to_string(2));
  EXPECT_EQ(road_geometry->branch_point(2)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(2)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(3)->id().id,
            std::string("bp:") + std::to_string(3));
  EXPECT_EQ(road_geometry->branch_point(3)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(3)->GetBSide()->size(), 0);

  // Check the brach point assigment regarding the lanes
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
GTEST_TEST(RNDFBuilder, UShapedLane) {
  std::unique_ptr<Builder> builder =
      std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);

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
    connected_lanes.push_back(
        Connection(std::to_string(1), waypoints, kLaneWidth, false));
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
    connected_lanes.push_back(
        Connection(std::to_string(2), waypoints, kLaneWidth, false));
  }

  auto bounding_box =
      std::make_tuple<ignition::math::Vector3d, ignition::math::Vector3d>(
          ignition::math::Vector3d(-40.0, 0.0, 0.0),
          ignition::math::Vector3d(40., 50.0, 0.0));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"UShapedLane"});
  EXPECT_NE(road_geometry, nullptr);

  // Check junction, segments and lanes. Lane naming implies direction
  // so checking correct for right naming implies proper direction inference.
  std::vector<std::tuple<std::string, std::string, std::string> > name_set = {
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

  std::string junction_name, segment_name, lane_name;
  for (auto& names : name_set) {
    std::tie(junction_name, segment_name, lane_name) = names;
    int junction_id = FindJunction(*road_geometry, junction_name);
    EXPECT_NE(junction_id, -1);
    const api::Junction* junction = road_geometry->junction(junction_id);
    EXPECT_TRUE(junction != nullptr);
    EXPECT_EQ(junction->num_segments(), 1);
    const api::Segment* segment = junction->segment(0);
    EXPECT_TRUE(segment != nullptr);
    EXPECT_EQ(segment->id().id, segment_name);
    EXPECT_EQ(segment->num_lanes(), 1);
    const api::Lane* lane = segment->lane(0);
    EXPECT_TRUE(lane != nullptr);
    EXPECT_EQ(lane->id().id, lane_name);
  }
}

//          1.1.1      1.1.2       1.1.3
//          * > > > > > * > > > > > *
//  1.2.1   1.2.2  1.2.3    1.2.4
//  * > > > * > > > * > > > > *
//              1.3.2                    1.3.1
//              * < < < < < < < < < < < < *
GTEST_TEST(RNDFBuilder, MultilaneLane) {
  const api::RBounds single_lane_bounds(-kLaneWidth / 2.0, kLaneWidth / 2.0);
  const api::RBounds two_lane_bounds_left(-kLaneWidth / 2.0,
                                          kLaneWidth / 2.0 + 10.0);
  const api::RBounds two_lane_bounds_right(-kLaneWidth / 2.0 - 10.0,
                                           kLaneWidth / 2.0);

  std::unique_ptr<Builder> builder =
      std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);
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
    Connection connection(std::to_string(1), waypoints, kLaneWidth, false);
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
    Connection connection(std::to_string(1), waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }

  {
    std::vector<DirectedWaypoint> waypoints(2, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(1, 3, 1));
    waypoints[0].set_position(ignition::math::Vector3d(40., -10.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(1, 3, 2));
    waypoints[1].set_position(ignition::math::Vector3d(5.0, -10.0, 0.0));
    Connection connection(std::to_string(1), waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }

  auto bounding_box =
      std::make_tuple<ignition::math::Vector3d, ignition::math::Vector3d>(
          ignition::math::Vector3d(0., -10., 0.),
          ignition::math::Vector3d(40., 10., 0.));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"MultilaneLane"});
  EXPECT_NE(road_geometry, nullptr);

  // Check road creation, naming, which lane is at both sides and bounds.
  EXPECT_EQ(road_geometry->num_junctions(), 6);
  EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id,
            std::string("s:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id,
            std::string("l:1.2.1-1.2.2"));
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

  EXPECT_EQ(road_geometry->junction(1)->id().id, std::string("j:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id,
            std::string("s:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->id().id,
            std::string("l:1.2.2-1.2.3"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(1)->id().id,
            std::string("l:1.1.1-1.1.5"));
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

  EXPECT_EQ(road_geometry->junction(2)->id().id, std::string("j:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id,
            std::string("s:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->id().id,
            std::string("l:1.2.3-1.2.5"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(1)->id().id,
            std::string("l:1.1.5-1.1.2"));
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

  EXPECT_EQ(road_geometry->junction(3)->id().id, std::string("j:1-0-3"));
  EXPECT_EQ(road_geometry->junction(3)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->id().id,
            std::string("s:1-0-3"));
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(0)->id().id,
            std::string("l:1.2.5-1.2.4"));
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(1)->id().id,
            std::string("l:1.1.2-1.1.6"));
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

  EXPECT_EQ(road_geometry->junction(4)->id().id, std::string("j:1-0-4"));
  EXPECT_EQ(road_geometry->junction(4)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->id().id,
            std::string("s:1-0-4"));
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->lane(0)->id().id,
            std::string("l:1.1.6-1.1.3"));
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

  EXPECT_EQ(road_geometry->junction(5)->id().id, std::string("j:1-1-0"));
  EXPECT_EQ(road_geometry->junction(5)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->id().id,
            std::string("s:1-1-0"));
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->lane(0)->id().id,
            std::string("l:1.3.1-1.3.2"));
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
GTEST_TEST(RNDFBuilder, MultilaneLaneCross) {
  std::unique_ptr<Builder> builder =
      std::make_unique<Builder>(kLinearTolerance, kAngularTolerance);
  auto bounding_box =
      std::make_tuple<ignition::math::Vector3d, ignition::math::Vector3d>(
          ignition::math::Vector3d(0., 0., 0.),
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
    Connection connection(std::to_string(1), waypoints, kLaneWidth, false);
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
    Connection connection(std::to_string(1), waypoints, kLaneWidth, false);
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
    Connection connection(std::to_string(2), waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }
  {
    std::vector<DirectedWaypoint> waypoints(2, DirectedWaypoint());
    waypoints[0].set_id(ignition::rndf::UniqueId(2, 2, 1));
    waypoints[0].set_position(ignition::math::Vector3d(10., 30.0, 0.0));
    waypoints[1].set_id(ignition::rndf::UniqueId(2, 2, 2));
    waypoints[1].set_position(ignition::math::Vector3d(0.0, 30.0, 0.0));
    Connection connection(std::to_string(2), waypoints, kLaneWidth, false);
    connected_lanes.push_back(connection);
  }
  builder->CreateSegmentConnections(2, &connected_lanes);

  builder->CreateConnection(kLaneWidth, ignition::rndf::UniqueId(1, 1, 2),
                            ignition::rndf::UniqueId(2, 2, 1));
  builder->CreateConnection(kLaneWidth, ignition::rndf::UniqueId(2, 1, 2),
                            ignition::rndf::UniqueId(1, 2, 2));
  builder->CreateConnection(kLaneWidth, ignition::rndf::UniqueId(2, 1, 2),
                            ignition::rndf::UniqueId(1, 1, 3));

  std::unique_ptr<const api::RoadGeometry> road_geometry =
      builder->Build({"MultilaneLaneCross"});
  EXPECT_NE(road_geometry, nullptr);
  EXPECT_EQ(road_geometry->num_junctions(), 11);

  // Here I check for the lane creation, naming, and bound coordinates
  int junction_id;
  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-0-0"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:1-0-0"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:1.1.1-1.1.2"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(20., 50.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(20.0, 40.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-0-1"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:1-0-1"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:1.1.2-1.1.3"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(20.0, 40.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(20.0, 10.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-0-2"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:1-0-2"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:1.1.3-1.1.4"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(20.0, 10.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(20.0, 0.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-1-0"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:1-1-0"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:1.2.1-1.2.2"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(30.0, 0.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(30.0, 30.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-1-1"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:1-1-1"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:1.2.2-1.2.3"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(30.0, 30.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(30.0, 50.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:2-0-0"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:2-0-0"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:2.1.1-2.1.2"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(0.0, 20.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:2-0-1"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:2-0-1"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:2.1.2-2.1.3"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(40.0, 20.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:2-1-0"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:2-1-0"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:2.2.1-2.2.2"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(10.0, 30.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(0.0, 30.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:1.1.2-2.2.1"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:1.1.2-2.2.1"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:1.1.2-2.2.1"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(20.0, 40.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(10.0, 30.0, 0.0), kLinearTolerance));
  }
  {
    junction_id = FindJunction(*road_geometry, std::string("j:2.1.2-1.2.2"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:2.1.2-1.2.2"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:2.1.2-1.2.2"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(30.0, 30.0, 0.0), kLinearTolerance));
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:2.1.2-1.1.3"));
    EXPECT_NE(junction_id, -1);
    EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
              std::string("s:2.1.2-1.1.3"));
    EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

    auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().id, std::string("l:2.1.2-1.1.3"));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
        api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        lane->ToGeoPosition(api::LanePosition(lane->length(), 0.0, 0.0)),
        api::GeoPosition(20.0, 10.0, 0.0), kLinearTolerance));
  }

  EXPECT_EQ(road_geometry->num_branch_points(), 12);
  // Checks for the branch points
  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-0-0"));
    auto branch_point = road_geometry->junction(junction_id)
                            ->segment(0)
                            ->lane(0)
                            ->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_NE(branch_point, nullptr);
    EXPECT_EQ(branch_point->GetASide()->size(), 1);
    EXPECT_EQ(branch_point->GetBSide()->size(), 2);
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-0-1"));
    auto branch_point = road_geometry->junction(junction_id)
                            ->segment(0)
                            ->lane(0)
                            ->GetBranchPoint(api::LaneEnd::kFinish);
    EXPECT_NE(branch_point, nullptr);
    EXPECT_EQ(branch_point->GetASide()->size(), 2);
    EXPECT_EQ(branch_point->GetBSide()->size(), 1);
  }

  {
    junction_id = FindJunction(*road_geometry, std::string("j:1-1-1"));
    auto branch_point = road_geometry->junction(junction_id)
                            ->segment(0)
                            ->lane(0)
                            ->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(branch_point, nullptr);
    EXPECT_EQ(branch_point->GetASide()->size(), 2);
    EXPECT_EQ(branch_point->GetBSide()->size(), 1);
  }
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
