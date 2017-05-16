#include "drake/automotive/maliput/monolane/road_geometry.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/test/monolane_test_utils.h"

namespace drake {
namespace maliput {
namespace monolane {

using api::RBounds;
using monolane::ArcOffset;

const double kVeryExact = 1e-11;
const double kWidth{2.};  // Lane and drivable width.

const api::Lane* GetLaneByJunctionId(const api::RoadGeometry& rg,
                                     const std::string& junction_id) {
  for (int i = 0; i < rg.num_junctions(); ++i) {
    if (rg.junction(i)->id().id == junction_id) {
      return rg.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

GTEST_TEST(MonolaneLanesTest, DoToRoadPosition) {
  // Define a serpentine road with multiple segments and branches.
  std::unique_ptr<monolane::Builder> rb(
      new monolane::Builder(RBounds(-kWidth, kWidth), RBounds(-kWidth, kWidth),
                            0.01, /* linear tolerance */
                            0.01 * M_PI /* angular tolerance */));

  // Initialize the road from the origin.
  const monolane::EndpointXy kOriginXy{0., 0., 0.};
  const monolane::EndpointZ kFlatZ{0., 0., 0., 0.};
  const monolane::Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  // Define the lanes and connections.
  const double kArcDeltaTheta{M_PI / 2.};
  const double kArcRadius{50.};
  const double kLength{50.};
  const auto& lane0 = rb->Connect(
      "lane0", kRoadOrigin, ArcOffset(kArcRadius, -kArcDeltaTheta), kFlatZ);
  const auto& lane1 = rb->Connect("lane1", lane0->end(), kLength, kFlatZ);
  const auto& lane2 = rb->Connect(
      "lane2", lane1->end(), ArcOffset(kArcRadius, kArcDeltaTheta), kFlatZ);
  rb->Connect("lane3a", lane2->end(), kLength, kFlatZ);
  rb->Connect("lane3b", lane2->end(), ArcOffset(kArcRadius, kArcDeltaTheta),
              kFlatZ);
  rb->Connect("lane3c", lane2->end(), ArcOffset(kArcRadius, -kArcDeltaTheta),
              kFlatZ);

  std::unique_ptr<const api::RoadGeometry> rg =
      rb->Build({"multi_lane_with_branches"});

  // Place a point at the middle of lane1.
  api::GeoPosition geo_pos{kArcRadius, -kArcRadius - kLength / 2., 0.};

  api::GeoPosition nearest_position{};
  double distance;
  api::RoadPosition actual_position =
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, &distance);

  // Expect to locate the point centered within lane1 (straight segment).
  EXPECT_LANE_NEAR(actual_position.pos,
                   (kLength / 2. /* s */, 0. /* r */, 0. /* h */), kVeryExact);
  EXPECT_EQ(actual_position.lane->id().id, "l:lane1");
  EXPECT_EQ(distance, 0.);
  EXPECT_GEO_NEAR(nearest_position, (geo_pos.x(), geo_pos.y(), geo_pos.z()),
                  kVeryExact);

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
  EXPECT_LANE_NEAR(actual_position.pos,
                   (kLength / 2. /* s */, kWidth /* r */, 0. /* h */),
                   kVeryExact);
  EXPECT_EQ(actual_position.lane->id().id, "l:lane1");
  EXPECT_EQ(distance, kWidth);
  EXPECT_GEO_NEAR(nearest_position,
                  (geo_pos.x() - kWidth, geo_pos.y(), geo_pos.z()), kVeryExact);

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
  EXPECT_LANE_NEAR(actual_position.pos,
                   (kLength / 2. /* s */, 0. /* r */, 0. /* h */), kVeryExact);
  EXPECT_EQ(actual_position.lane->id().id, "l:lane3a");
  EXPECT_EQ(distance, 0.);
  EXPECT_GEO_NEAR(nearest_position, (geo_pos.x(), geo_pos.y(), geo_pos.z()),
                  kVeryExact);

  // Place a point at the end of lane3b (arc segment).
  geo_pos =
      api::GeoPosition(2. * kArcRadius + kLength, -kArcRadius - kLength, 0.);

  actual_position =
      rg->ToRoadPosition(geo_pos, nullptr, &nearest_position, &distance);

  // Expect to locate the point at the end of lane3b.
  EXPECT_LANE_NEAR(actual_position.pos,
                   (kArcRadius * M_PI / 2. /* s */, 0. /* r */, 0. /* h */),
                   kVeryExact);
  EXPECT_EQ(actual_position.lane->id().id, "l:lane3b");
  EXPECT_EQ(distance, 0.);
  EXPECT_GEO_NEAR(nearest_position, (geo_pos.x(), geo_pos.y(), geo_pos.z()),
                  kVeryExact);

  // Supply a hint with a position at the start of lane3c to try and determine
  // the RoadPosition for a point at the end of lane3b.
  api::RoadPosition hint{GetLaneByJunctionId(*rg, "j:lane3c"), {0., 0., 0.}};
  actual_position =
      rg->ToRoadPosition(geo_pos, &hint, &nearest_position, &distance);

  // Expect to locate the point outside of lanes lane3c (and ongoing adjacent
  // lanes), since lane3b is not ongoing from lane3c.
  EXPECT_EQ(actual_position.lane->id().id, "l:lane3c");
  EXPECT_GT(distance, 0.);  // geo_pos is not within this lane.

  // Supply a hint with a position at the start of lane2 to try and determine
  // the RoadPosition for a point at the end of lane3b.
  hint = api::RoadPosition{GetLaneByJunctionId(*rg, "j:lane2"), {0., 0., 0.}};
  actual_position =
      rg->ToRoadPosition(geo_pos, &hint, &nearest_position, &distance);

  // Expect to traverse to lane3b (an ongoing lane) and then locate the point
  // within lane lane3b.
  EXPECT_EQ(actual_position.lane->id().id, "l:lane3b");
  EXPECT_EQ(distance, 0.);  // geo_pos is inside lane3b.
  EXPECT_GEO_NEAR(nearest_position, (geo_pos.x(), geo_pos.y(), geo_pos.z()),
                  kVeryExact);
}

GTEST_TEST(MonolaneLanesTest, HintWithDisconnectedLanes) {
  // Define a road with two disconnected, diverging lanes such that there are no
  // ongoing lanes.  This tests the pathological case when a `hint` is provided
  // in a topologically isolated lane, so the code returns the default road
  // position given by the hint.
  std::unique_ptr<monolane::Builder> rb(
      new monolane::Builder(RBounds(-kWidth, kWidth), RBounds(-kWidth, kWidth),
                            0.01, /* linear tolerance */
                            0.01 * M_PI /* angular tolerance */));

  // Initialize the road from the origin.
  const monolane::EndpointXy kOriginXy0{0., 0., 0.};
  const monolane::EndpointXy kOriginXy1{0., 100., 0.};
  const monolane::EndpointZ kFlatZ{0., 0., 0., 0.};
  const monolane::Endpoint kRoadOrigin0{kOriginXy0, kFlatZ};
  const monolane::Endpoint kRoadOrigin1{kOriginXy1, kFlatZ};

  // Define the lanes and connections.
  rb->Connect("lane0", kRoadOrigin0, ArcOffset(50., -M_PI / 2.), kFlatZ);
  rb->Connect("lane1", kRoadOrigin1, ArcOffset(50., M_PI / 2.), kFlatZ);

  std::unique_ptr<const api::RoadGeometry> rg =
      rb->Build({"disconnected_lanes"});

  // Place a point at the middle of lane0.
  api::GeoPosition geo_pos{50. * std::sqrt(2.) / 2., -50. * std::sqrt(2.) / 2.,
                           0.};

  // Supply a hint with a position at the start of lane1.
  double distance;
  api::RoadPosition hint =
      api::RoadPosition{GetLaneByJunctionId(*rg, "j:lane1"), {0., 0., 0.}};
  api::RoadPosition actual_position{};
  EXPECT_NO_THROW(actual_position =
                      rg->ToRoadPosition(geo_pos, &hint, nullptr, &distance));
  EXPECT_EQ(actual_position.lane->id().id, "l:lane1");  // The search is
                                                        // confined to lane1.
  EXPECT_GT(distance, 0.);  // lane1 does not contain the point.
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
