/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/road_geometry.h"
/* clang-format on */

#include <cmath>
#include <map>
#include <tuple>
#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/multilane/builder.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

using api::RBounds;
using api::HBounds;
using multilane::ArcOffset;
using Which = api::LaneEnd::Which;

const double kVeryExact{1e-11};
const double kWidth{2.};  // Lane and drivable width.
const double kHeight{5.};  // Elevation bound.

const api::Lane* GetLaneByJunctionId(const api::RoadGeometry& rg,
                                     const std::string& junction_id,
                                     int segment_index, int lane_index) {
  DRAKE_DEMAND(segment_index >= 0);
  DRAKE_DEMAND(lane_index >= 0);

  for (int i = 0; i < rg.num_junctions(); ++i) {
    if (rg.junction(i)->id() == api::JunctionId(junction_id)) {
      if (segment_index >= rg.junction(i)->num_segments()) {
        throw std::runtime_error(
            "Segment index is greater than available segment number.");
      }
      if (lane_index >= rg.junction(i)->segment(segment_index)->num_lanes()) {
        throw std::runtime_error(
            "Lane index is greater than available lane number.");
      }
      return rg.junction(i)->segment(segment_index)->lane(lane_index);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

const api::Lane* GetLaneByJunctionId(const api::RoadGeometry& rg,
                                     const std::string& junction_id) {
  return GetLaneByJunctionId(rg, junction_id, 0, 0);
}

GTEST_TEST(MultilaneLanesTest, DoToRoadPosition) {
  // Define a serpentine road with multiple segments and branches.
  auto rb = multilane::BuilderFactory().Make(
      2. * kWidth, HBounds(0., kHeight), 0.01, /* linear tolerance */
      0.01 * M_PI /* angular tolerance */);

  // Initialize the road from the origin.
  const multilane::EndpointXy kOriginXy{0., 0., 0.};
  const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
  const multilane::Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  // Define the lanes and connections.
  const double kArcDeltaTheta{M_PI / 2.};
  const double kArcRadius{50.};
  const double kLength{50.};
  const double kOneLane{1};
  const double kZeroR0{0.};
  const double kNoShoulder{0.};

  const int kRefLane{0};
  const LaneLayout kMonolaneLayout(kNoShoulder, kNoShoulder, kOneLane, kRefLane,
                                   kZeroR0);

  const auto& lane0 =
      rb->Connect("lane0", kMonolaneLayout,
                  StartReference().at(kRoadOrigin, Direction::kForward),
                  ArcOffset(kArcRadius, -kArcDeltaTheta),
                  EndReference().z_at(kFlatZ, Direction::kForward));

  const auto& lane1 = rb->Connect(
      "lane1", kMonolaneLayout,
      StartReference().at(*lane0, Which::kFinish, Direction::kForward),
      LineOffset(kLength), EndReference().z_at(kFlatZ, Direction::kForward));

  const auto& lane2 = rb->Connect(
      "lane2", kMonolaneLayout,
      StartReference().at(*lane1, Which::kFinish, Direction::kForward),
      ArcOffset(kArcRadius, kArcDeltaTheta),
      EndReference().z_at(kFlatZ, Direction::kForward));

  rb->Connect("lane3a", kMonolaneLayout,
              StartReference().at(*lane2, Which::kFinish, Direction::kForward),
              LineOffset(kLength),
              EndReference().z_at(kFlatZ, Direction::kForward));

  rb->Connect("lane3b", kMonolaneLayout,
              StartReference().at(*lane2, Which::kFinish, Direction::kForward),
              ArcOffset(kArcRadius, kArcDeltaTheta),
              EndReference().z_at(kFlatZ, Direction::kForward));

  rb->Connect("lane3c", kMonolaneLayout,
              StartReference().at(*lane2, Which::kFinish, Direction::kForward),
              ArcOffset(kArcRadius, -kArcDeltaTheta),
              EndReference().z_at(kFlatZ, Direction::kForward));

  std::unique_ptr<const api::RoadGeometry> rg =
      rb->Build(api::RoadGeometryId{"multi_lane_with_branches"});

  // Place a point at the middle of lane1.
  api::GeoPosition geo_pos{kArcRadius, -kArcRadius - kLength / 2., 0.};

  api::GeoPosition nearest_position{};
  double distance{};
  api::RoadPosition actual_position =
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, &distance);

  // Expect to locate the point centered within lane1 (straight segment).
  EXPECT_TRUE(api::test::IsLanePositionClose(
      actual_position.pos,
      api::LanePosition(kLength / 2. /* s */, 0. /* r */, 0. /* h */),
      kVeryExact));
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane1_0"));
  EXPECT_EQ(distance, 0.);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      nearest_position, api::GeoPosition(geo_pos.x(), geo_pos.y(), geo_pos.z()),
      kVeryExact));

  // Tests the integrity of ToRoadPosition() with various other null argument
  // combinations for the case where the point is within a lane.
  EXPECT_NO_THROW(
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, nullptr));
  EXPECT_NO_THROW(rg->ToRoadPosition(geo_pos, nullptr, nullptr, &distance));
  EXPECT_NO_THROW(rg->ToRoadPosition(geo_pos, nullptr, nullptr, nullptr));

  // Place a point halfway to the end of lane1, just to the outside (left side)
  // of the lane bounds.
  geo_pos = api::GeoPosition(kArcRadius + 2. * kWidth,
                             -kArcRadius - kLength / 2., 0.);

  actual_position =
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, &distance);

  // Expect to locate the point just outside (to the left) of lane1, by an
  // amount kWidth.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      actual_position.pos,
      api::LanePosition(kLength / 2. /* s */, kWidth /* r */, 0. /* h */),
      kVeryExact));
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane1_0"));
  EXPECT_EQ(distance, kWidth);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      nearest_position,
      api::GeoPosition(geo_pos.x() - kWidth, geo_pos.y(), geo_pos.z()),
      kVeryExact));

  // Tests the integrity of ToRoadPosition() with various other null argument
  // combinations for the case where the point is outside all lanes.
  EXPECT_NO_THROW(
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, nullptr));
  EXPECT_NO_THROW(rg->ToRoadPosition(geo_pos, nullptr, nullptr, &distance));
  EXPECT_NO_THROW(rg->ToRoadPosition(geo_pos, nullptr, nullptr, nullptr));

  // Place a point at the middle of lane3a (straight segment).
  geo_pos = api::GeoPosition(2. * kArcRadius + kLength / 2.,
                             -2. * kArcRadius - kLength, 0.);

  actual_position =
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, &distance);

  // Expect to locate the point centered within lane3a.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      actual_position.pos,
      api::LanePosition(kLength / 2. /* s */, 0. /* r */, 0. /* h */),
      kVeryExact));
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane3a_0"));
  EXPECT_EQ(distance, 0.);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      nearest_position, api::GeoPosition(geo_pos.x(), geo_pos.y(), geo_pos.z()),
      kVeryExact));

  // Place a point high above the middle of lane3a (straight segment).
  geo_pos = api::GeoPosition(2. * kArcRadius + kLength / 2.,
                             -2. * kArcRadius - kLength,
                             50.);

  actual_position =
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, &distance);

  // Expect to locate the point centered above lane3a.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      actual_position.pos,
      api::LanePosition(kLength / 2. /* s */, 0. /* r */, kHeight /* h */),
      kVeryExact));
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane3a_0"));
  EXPECT_EQ(distance, 50. - kHeight);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      nearest_position, api::GeoPosition(geo_pos.x(), geo_pos.y(), kHeight),
      kVeryExact));

  // Place a point at the end of lane3b (arc segment).
  geo_pos =
      api::GeoPosition(2. * kArcRadius + kLength, -kArcRadius - kLength, 0.);

  actual_position =
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, &distance);

  // Expect to locate the point at the end of lane3b.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      actual_position.pos,
      api::LanePosition(kArcRadius * M_PI / 2. /* s */, 0. /* r */, 0. /* h */),
      kVeryExact));
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane3b_0"));
  EXPECT_EQ(distance, 0.);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      nearest_position, api::GeoPosition(geo_pos.x(), geo_pos.y(), geo_pos.z()),
      kVeryExact));

  // Supply a hint with a position at the start of lane3c to try and determine
  // the RoadPosition for a point at the end of lane3b.
  api::RoadPosition hint{GetLaneByJunctionId(*rg, "j:lane3c"), {0., 0., 0.}};
  actual_position =
      rg->ToRoadPosition(geo_pos, &hint, &nearest_position, &distance);

  // Expect to locate the point outside of lanes lane3c (and ongoing adjacent
  // lanes), since lane3b is not ongoing from lane3c.
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane3c_0"));
  EXPECT_GT(distance, 0.);  // geo_pos is not within this lane.

  // Supply a hint with a position at the start of lane2 to try and determine
  // the RoadPosition for a point at the end of lane3b.
  hint = api::RoadPosition{GetLaneByJunctionId(*rg, "j:lane2"), {0., 0., 0.}};
  actual_position =
      rg->ToRoadPosition(geo_pos, &hint, &nearest_position, &distance);

  // Expect to traverse to lane3b (an ongoing lane) and then locate the point
  // within lane lane3b.
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane3b_0"));
  EXPECT_EQ(distance, 0.);  // geo_pos is inside lane3b.
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      nearest_position, api::GeoPosition(geo_pos.x(), geo_pos.y(), geo_pos.z()),
      kVeryExact));
}

GTEST_TEST(MultilaneLanesTest, HintWithDisconnectedLanes) {
  // Define a road with two disconnected, diverging lanes such that there are no
  // ongoing lanes.  This tests the pathological case when a `hint` is provided
  // in a topologically isolated lane, so the code returns the default road
  // position given by the hint.
  auto rb = multilane::BuilderFactory().Make(
      2. * kWidth, HBounds(0., kHeight), 0.01, /* linear tolerance */
      0.01 * M_PI /* angular tolerance */);

  // Initialize the road from the origin.
  const multilane::EndpointXy kOriginXy0{0., 0., 0.};
  const multilane::EndpointXy kOriginXy1{0., 100., 0.};
  const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
  const multilane::Endpoint kRoadOrigin0{kOriginXy0, kFlatZ};
  const multilane::Endpoint kRoadOrigin1{kOriginXy1, kFlatZ};
  const int kOneLane{1};
  const int kRefLane{0};
  const double kZeroR0{0.};
  const double kNoShoulder{0.};

  // Define the lanes and connections.
  const LaneLayout kMonolaneLayout(kNoShoulder, kNoShoulder, kOneLane, kRefLane,
                                   kZeroR0);

  rb->Connect("lane0", kMonolaneLayout,
              StartReference().at(kRoadOrigin0, Direction::kForward),
              ArcOffset(50., -M_PI / 2.),
              EndReference().z_at(kFlatZ, Direction::kForward));

  rb->Connect("lane1", kMonolaneLayout,
              StartReference().at(kRoadOrigin1, Direction::kForward),
              ArcOffset(50., M_PI / 2.),
              EndReference().z_at(kFlatZ, Direction::kForward));

  std::unique_ptr<const api::RoadGeometry> rg =
      rb->Build(api::RoadGeometryId{"disconnected_lanes"});

  // Place a point at the middle of lane0.
  api::GeoPosition geo_pos{50. * std::sqrt(2.) / 2., -50. * std::sqrt(2.) / 2.,
                           0.};

  // Supply a hint with a position at the start of lane1.
  double distance{};
  api::RoadPosition hint =
      api::RoadPosition{GetLaneByJunctionId(*rg, "j:lane1"), {0., 0., 0.}};
  api::RoadPosition actual_position{};
  EXPECT_NO_THROW(actual_position =
                      rg->ToRoadPosition(geo_pos, &hint, nullptr, &distance));
  // The search is confined to lane1.
  EXPECT_EQ(actual_position.lane->id(), api::LaneId("l:lane1_0"));
  // lane1 does not contain the point.
  EXPECT_GT(distance, 0.);
}

// Tests different api::Geoposition in the following RoadGeometry without a
// hint.
//
//        ^ +r, +y
//        |
//        |                 g
//        ------------------------------------
//        |                 f                |           Left shoulder
//        ------------------------------------
//        |                 e                |           l:2
//        |                                  |
//        ------------------------------------
//        |                                  |           l:1
//        |                 d                |
//        ------------------c-----------------
// (0,0,0)|__________________________________|_____> +s  l:0
//        |                                  |       +x
//        ------------------------------------
//        |                 b                |           Right shoulder
//        ------------------------------------
//                          a
//
// Letters, such as `a`, `b`, etc. are the api::GeoPositions to test.
GTEST_TEST(MultilaneLanesTest, MultipleLineLaneSegmentWithoutHint) {
  const HBounds kElevationBounds{0., kHeight};
  const double kLinearTolerance{kVeryExact};
  const double kAngularTolerance{0.01 * M_PI};
  const double kLaneWidth{2. * kWidth};

  auto builder = multilane::BuilderFactory().Make(
      kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance);

  // Initialize the road from the origin.
  const EndpointZ kFlatZ{0., 0., 0., 0.};
  const double kZeroZ{0.};
  const Endpoint kRoadOrigin{{0., 0., 0.}, kFlatZ};
  const double kLength{10.};
  const double kHalfLength{0.5 * kLength};
  const int kThreeLanes{3};
  const int kRefLane{0};
  const double kZeroR0{0.};
  const double kShoulder{1.0};
  const LaneLayout kThreeLaneLayout(kShoulder, kShoulder, kThreeLanes, kRefLane,
                                    kZeroR0);
  // Creates a simple 3-line-lane segment road.
  builder->Connect("s0", kThreeLaneLayout,
                   StartReference().at(kRoadOrigin, Direction::kForward),
                   LineOffset(kLength),
                   EndReference().z_at(kFlatZ, Direction::kForward));
  std::unique_ptr<const api::RoadGeometry> rg =
      builder->Build(api::RoadGeometryId{"multi-lane-line-segment"});

  // Prepares the truth table to match different api::GeoPositions into
  // api::RoadPositions.
  const api::Lane* kFirstLane = GetLaneByJunctionId(*rg, "j:s0", 0, 0);
  const api::Lane* kSecondLane = GetLaneByJunctionId(*rg, "j:s0", 0, 1);
  const api::Lane* kThirdLane = GetLaneByJunctionId(*rg, "j:s0", 0, 2);

  // <Geo point - Expected road pos - Nearest pos - Hint - Distance >
  const std::vector<std::tuple<api::GeoPosition, api::RoadPosition,
                               api::GeoPosition, api::RoadPosition, double>>
      truth_vector{
          std::make_tuple<api::GeoPosition, api::RoadPosition, api::GeoPosition,
                          api::RoadPosition, double>(  // a
              {kHalfLength, -kLaneWidth, kZeroZ},
              {kFirstLane, {kHalfLength, -0.75 * kLaneWidth, kZeroZ}},
              {kHalfLength, -0.75 * kLaneWidth, kZeroZ}, {kFirstLane, {}},
              0.25 * kLaneWidth),
          std::make_tuple<api::GeoPosition, api::RoadPosition, api::GeoPosition,
                          api::RoadPosition, double>(  // b
              {kHalfLength, -0.625 * kLaneWidth, kZeroZ},
              {kFirstLane, {kHalfLength, -0.625 * kLaneWidth, kZeroZ}},
              {kHalfLength, -0.625 * kLaneWidth, kZeroZ}, {kFirstLane, {}}, 0.),
          std::make_tuple<api::GeoPosition, api::RoadPosition, api::GeoPosition,
                          api::RoadPosition, double>(  // c
              {kHalfLength, 0.5 * kLaneWidth, kZeroZ},
              {kSecondLane, {kHalfLength, -0.5 * kLaneWidth, kZeroZ}},
              {kHalfLength, 0.5 * kLaneWidth, kZeroZ}, {kSecondLane, {}}, 0.),
          std::make_tuple<api::GeoPosition, api::RoadPosition, api::GeoPosition,
                          api::RoadPosition, double>(  // d
              {kHalfLength, 0.775 * kLaneWidth, kZeroZ},
              {kSecondLane, {kHalfLength, -0.225 * kLaneWidth, kZeroZ}},
              {kHalfLength, 0.775 * kLaneWidth, kZeroZ}, {kSecondLane, {}}, 0.),
          std::make_tuple<api::GeoPosition, api::RoadPosition, api::GeoPosition,
                          api::RoadPosition, double>(  // e
              {kHalfLength, 2.075 * kLaneWidth, kZeroZ},
              {kThirdLane, {kHalfLength, 0.075 * kLaneWidth, kZeroZ}},
              {kHalfLength, 2.075 * kLaneWidth, kZeroZ}, {kThirdLane, {}}, 0.),
          std::make_tuple<api::GeoPosition, api::RoadPosition, api::GeoPosition,
                          api::RoadPosition, double>(  // f
              {kHalfLength, 2.625 * kLaneWidth, kZeroZ},
              {kThirdLane, {kHalfLength, 0.625 * kLaneWidth, kZeroZ}},
              {kHalfLength, 2.625 * kLaneWidth, kZeroZ}, {kThirdLane, {}}, 0.),
          std::make_tuple<api::GeoPosition, api::RoadPosition, api::GeoPosition,
                          api::RoadPosition, double>(  // g
              {kHalfLength, 3. * kLaneWidth, kZeroZ},
              {kThirdLane, {kHalfLength, 0.75 * kLaneWidth, kZeroZ}},
              {kHalfLength, 2.75 * kLaneWidth, kZeroZ}, {kThirdLane, {}},
              0.25 * kLaneWidth),
      };

  double distance{};
  api::RoadPosition test_road_position{};
  api::GeoPosition nearest_position{};
  // Evaluates the truth table without a hint.
  for (const auto truth_value : truth_vector) {
    EXPECT_NO_THROW(test_road_position =
                        rg->ToRoadPosition(std::get<0>(truth_value), nullptr,
                                           &nearest_position, &distance));
    EXPECT_EQ(test_road_position.lane->id(),
              std::get<1>(truth_value).lane->id());
    EXPECT_TRUE(api::test::IsLanePositionClose(test_road_position.pos,
                                               std::get<1>(truth_value).pos,
                                               kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        nearest_position, std::get<2>(truth_value), kLinearTolerance));
    EXPECT_NEAR(distance, std::get<4>(truth_value), kLinearTolerance);
  }

  // Evaluates the truth table with a hint.
  for (const auto truth_value : truth_vector) {
    EXPECT_NO_THROW(test_road_position = rg->ToRoadPosition(
                        std::get<0>(truth_value), &(std::get<3>(truth_value)),
                        &nearest_position, &distance));
    EXPECT_EQ(test_road_position.lane->id(),
              std::get<1>(truth_value).lane->id());
    EXPECT_TRUE(api::test::IsLanePositionClose(test_road_position.pos,
                                               std::get<1>(truth_value).pos,
                                               kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        nearest_position, std::get<2>(truth_value), kLinearTolerance));
    EXPECT_NEAR(distance, std::get<4>(truth_value), kLinearTolerance);
  }
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
