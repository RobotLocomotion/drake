/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/loader.h"
/* clang-format on */

#include <cmath>
#include <map>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/multilane/lane.h"

namespace drake {
namespace maliput {
namespace multilane {

// TODO(agalbachicar)    Missing tests for non-zero EndpointZ and multi-lane
//                       segments.

// TODO(agalbachicar)    Missing tests for "reverse" semantic in "start",
//                       "explicit_end" and "z_end".

// TODO(agalbachicar)    Missing tests that use "explicit_end".

// Checks that the minimal YAML passes with an empty RoadGeometry.
GTEST_TEST(MultilaneLoaderTest, MinimalCorrectYaml) {
  const double kVeryExact{1e-12};
  const std::string kMultilaneYaml =
      "maliput_multilane_builder:\n"
      "  id: \"empty_road_geometry\"\n"
      "  lane_width: 4\n"
      "  left_shoulder: 1\n"
      "  right_shoulder: 2\n"
      "  elevation_bounds: [0, 5]\n"
      "  position_precision: 0.01\n"
      "  orientation_precision: 0.5\n"
      "  points: {}\n"
      "  connections: {}\n"
      "  groups: {}\n";

  std::unique_ptr<const api::RoadGeometry> rg = Load(kMultilaneYaml);
  EXPECT_NE(rg, nullptr);
  EXPECT_EQ(rg->id().string(), "empty_road_geometry");
  EXPECT_EQ(rg->linear_tolerance(), 0.01);
  EXPECT_NEAR(rg->angular_tolerance(), 0.5 / 180. * M_PI, kVeryExact);
  EXPECT_EQ(rg->num_junctions(), 0);
  EXPECT_EQ(rg->num_branch_points(), 0);
}

class MultilaneLoaderSingleLaneSegmentTest : public ::testing::Test {
 protected:
  const double kVeryExact{1e-12};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 / 180. * M_PI};
  const int kNumBranchPoints{2};
  const int kNumJunctions{1};
  const int kNumSegments{1};
  const int kNumLanes{1};
  const std::string kRoadGeometryName{"single_lane_segment"};
  const std::string kFirstJunctionName{"j:s1"};
  const std::string kFirstSegmentName{"s:s1"};
  const std::string kFirstLaneName{"l:s1_0"};
  const api::RBounds kLaneBounds{-2., 2.};
  const api::RBounds kDriveableBounds{-5.5, 5.};
  const api::HBounds kElevationBounds{0., 5.};
  const double kR0{2.1};
  const std::string kSingleLaneLineMultilaneYaml{
      "maliput_multilane_builder:\n"
      "  id: \"single_lane_segment\"\n"
      "  lane_width: 4\n"
      "  left_shoulder: 1\n"
      "  right_shoulder: 2\n"
      "  elevation_bounds: [0, 5]\n"
      "  position_precision: 0.01\n"
      "  orientation_precision: 0.5\n"
      "  points:\n"
      "    start:\n"
      "      xypoint: [20, 30, -45]\n"
      "      zpoint: [1, 1, 0, 0]\n"
      "  connections:\n"
      "    s1:\n"
      "      left_shoulder: 3\n"
      "      right_shoulder: 3.5\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"points.start\"]\n"
      "      length: 100\n"
      "      z_end: [\"ref\", [10, 1, 0, 0]]\n"
      "  groups: {}\n"};
  const std::string kSingleLaneArcMultilaneYaml{
      "maliput_multilane_builder:\n"
      "  id: \"single_lane_segment\"\n"
      "  lane_width: 4\n"
      "  left_shoulder: 1\n"
      "  right_shoulder: 2\n"
      "  elevation_bounds: [0, 5]\n"
      "  position_precision: 0.01\n"
      "  orientation_precision: 0.5\n"
      "  points:\n"
      "    start:\n"
      "      xypoint: [20, 30, -45]\n"
      "      zpoint: [1, 1, 0, 0]\n"
      "  connections:\n"
      "    s1:\n"
      "      left_shoulder: 3\n"
      "      right_shoulder: 3.5\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"points.start\"]\n"
      "      arc: [14.142, 90]\n"
      "      z_end: [\"ref\", [10, 1, 0, 0]]\n"
      "  groups: {}\n"};
};

// Checks the metadata of two RoadGeometries of a line segment.
TEST_F(MultilaneLoaderSingleLaneSegmentTest, SingleLaneSegmentMetadataChecks) {
  auto check_road_geometry_metadata = [&](const api::RoadGeometry& rg) {
    EXPECT_EQ(rg.id().string(), kRoadGeometryName);
    EXPECT_EQ(rg.linear_tolerance(), kLinearTolerance);
    EXPECT_NEAR(rg.angular_tolerance(), kAngularTolerance, kVeryExact);
    EXPECT_EQ(rg.num_junctions(), kNumJunctions);
    EXPECT_EQ(rg.num_branch_points(), kNumBranchPoints);
    EXPECT_EQ(rg.junction(0)->id().string(), kFirstJunctionName);
    EXPECT_EQ(rg.junction(0)->num_segments(), kNumSegments);
    EXPECT_EQ(rg.junction(0)->segment(0)->id().string(), kFirstSegmentName);
    EXPECT_EQ(rg.junction(0)->segment(0)->num_lanes(), kNumLanes);

    const api::Lane* lane = rg.junction(0)->segment(0)->lane(0);
    EXPECT_EQ(lane->id().string(), kFirstLaneName);

    // As it is the only lane, no other should be at its left nor right side.
    EXPECT_EQ(lane->to_left(), nullptr);
    EXPECT_EQ(lane->to_right(), nullptr);

    // Checks on the branch points at the extents of the lane.
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kStart)->GetASide()->size(),
              1);
    EXPECT_EQ(
        lane->GetBranchPoint(api::LaneEnd::kStart)->GetASide()->get(0).lane,
        lane);
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kStart)->GetBSide()->size(),
              0);
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kFinish)->GetASide()->size(),
              1);
    EXPECT_EQ(
        lane->GetBranchPoint(api::LaneEnd::kFinish)->GetASide()->get(0).lane,
        lane);
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kFinish)->GetBSide()->size(),
              0);
  };

  std::unique_ptr<const api::RoadGeometry> line_dut =
      Load(kSingleLaneLineMultilaneYaml);
  check_road_geometry_metadata(*line_dut);

  std::unique_ptr<const api::RoadGeometry> arc_dut =
      Load(kSingleLaneArcMultilaneYaml);
  check_road_geometry_metadata(*arc_dut);
}

// Checks the construction of a line segment.
TEST_F(MultilaneLoaderSingleLaneSegmentTest, SingleLineSegmentGeometry) {
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(kSingleLaneLineMultilaneYaml);
  EXPECT_NE(rg, nullptr);

  const Lane* lane =
      static_cast<const Lane*>(rg->junction(0)->segment(0)->lane(0));

  // Offset assignment.
  EXPECT_EQ(lane->r0(), kR0);

  // Bounds assignment. Note that driveable bounds are overridden by connection
  // specific properties.
  EXPECT_TRUE(api::test::IsRBoundsClose(lane->lane_bounds(0.), kLaneBounds,
                                        kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(lane->driveable_bounds(0.),
                                        kDriveableBounds, kVeryExact));
  EXPECT_TRUE(api::test::IsHBoundsClose(lane->elevation_bounds(0., 0.),
                                        kElevationBounds, kVeryExact));

  // Checks for elevation and superelevation polynomial.
  EXPECT_NEAR(lane->elevation().a(), 0.01, kVeryExact);
  EXPECT_NEAR(lane->elevation().b(), 1., kVeryExact);
  EXPECT_NEAR(lane->elevation().c(), -2.73, kVeryExact);
  EXPECT_NEAR(lane->elevation().d(), 1.82, kVeryExact);
  EXPECT_NEAR(lane->superelevation().a(), 0., kVeryExact);
  EXPECT_NEAR(lane->superelevation().b(), 0., kVeryExact);
  EXPECT_NEAR(lane->superelevation().c(), 0., kVeryExact);
  EXPECT_NEAR(lane->superelevation().d(), 0., kVeryExact);

  // The following length value comes from computing the in Octave the result of
  // LineRoadCurve::s_from_p(1, kR0). This is because, RoadCurve are not good at
  // computing the arc length of roads whose elevation polynomial order is other
  // than linear.
  const double length = 100.404183179786;
  EXPECT_NEAR(lane->length(), length, kVeryExact);
  // Positions over z=0 plane are computed by translating r0 start endpoint in
  // the r direction (+π/2 to the heading) and then translating start coordinate
  // planar length (100 m) in with heading angle. Height values are those
  // expressed in EndpointZ information.
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      lane->ToGeoPosition({0., 0., 0.}),
      {21.4849242404918, 31.4849242404918, 1.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      lane->ToGeoPosition({length, 0., 0.}),
      {92.1956023591465, -39.2257538781630, 10.}, kVeryExact));
}

// Checks the construction of an arc segment.
TEST_F(MultilaneLoaderSingleLaneSegmentTest, SingleLaneArcSegment) {
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(kSingleLaneArcMultilaneYaml);
  EXPECT_NE(rg, nullptr);

  const Lane* lane =
      static_cast<const Lane*>(rg->junction(0)->segment(0)->lane(0));

  // Offset assignment.
  EXPECT_EQ(lane->r0(), kR0);

  // Bounds assignment. Note that driveable bounds are overridden by connection
  // specific properties.
  EXPECT_TRUE(api::test::IsRBoundsClose(lane->lane_bounds(0.), kLaneBounds,
                                        kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(lane->driveable_bounds(0.),
                                        kDriveableBounds, kVeryExact));
  EXPECT_TRUE(api::test::IsHBoundsClose(lane->elevation_bounds(0., 0.),
                                        kElevationBounds, kVeryExact));

  // Checks for elevation and superelevation polynomial.
  EXPECT_NEAR(lane->elevation().a(), 0.0450162475157390, kVeryExact);
  EXPECT_NEAR(lane->elevation().b(), 1., kVeryExact);
  EXPECT_NEAR(lane->elevation().c(), -1.78456131707505, kVeryExact);
  EXPECT_NEAR(lane->elevation().d(), 1.18970754471670, kVeryExact);
  EXPECT_NEAR(lane->superelevation().a(), 0., kVeryExact);
  EXPECT_NEAR(lane->superelevation().b(), 0., kVeryExact);
  EXPECT_NEAR(lane->superelevation().c(), 0., kVeryExact);
  EXPECT_NEAR(lane->superelevation().d(), 0., kVeryExact);

  // The following length value comes from computing the in Octave the result of
  // ArcRoadCurve::s_from_p(1, kR0). This is because, RoadCurve are not good at
  // computing the arc length of roads whose elevation polynomial order is other
  // than linear.
  const double length = 20.9474879459049;
  EXPECT_NEAR(lane->length(), length, kVeryExact);
  // Positions over z=0 plane are computed by translating r0 start endpoint in
  // the r direction (+π/2 to the heading) and then translating start coordinate
  // planar length (100 m) in with heading angle. Height values are those
  // expressed in EndpointZ information.
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      lane->ToGeoPosition({0., 0., 0.}),
      {21.4849242404918, 31.4849242404918, 1.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      lane->ToGeoPosition({length, 0., 0.}),
      {38.5148839585886, 31.4849242404918, 10.}, kVeryExact));
}

class MultilaneLoaderSingleLaneSegmentConnectionTest : public ::testing::Test {
 protected:
  const double kVeryExact{1e-12};
  const double kLinearTolerance{0.01};
  const double kAngularTolerance{0.5 / 180. * M_PI};
  const int kNumBranchpoints{3};
  const int kNumJunctions{2};
  const int kNumSegments{1};
  const int kNumLanes{1};
  const std::string kRoadGeometryName{"two_connected_segments"};
  const std::string kFirstJunctionName{"j:s1"};
  const std::string kFirstSegmentName{"s:s1"};
  const std::string kFirstLaneName{"l:s1_0"};
  const std::string kSecondJunctionName{"j:s1"};
  const std::string kSecondSegmentName{"s:s2"};
  const std::string kSecondLaneName{"l:s2_0"};
  const std::string kMultilaneLineSegmentsYaml{
      "maliput_multilane_builder:\n"
      "  id: \"two_connected_segments\"\n"
      "  lane_width: 4\n"
      "  left_shoulder: 3\n"
      "  right_shoulder: 3.5\n"
      "  elevation_bounds: [0, 5]\n"
      "  position_precision: 0.01\n"
      "  orientation_precision: 0.5\n"
      "  points:\n"
      "    start:\n"
      "      xypoint: [20, 30, -45]\n"
      "      zpoint: [1, 1, 0, 0]\n"
      "  connections:\n"
      "    s1:\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"points.start\"]\n"
      "      length: 100\n"
      "      z_end: [\"ref\", [10, 1, 0, 0]]\n"
      "    s2:\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"connections.s1.end\"]\n"
      "      length: 100\n"
      "      z_end: [\"ref\", [10, 0, 0, 0]]\n"
      "  groups: {}\n"};
  const std::string kMultilaneArcSegmentsYaml{
      "maliput_multilane_builder:\n"
      "  id: \"two_connected_segments\"\n"
      "  lane_width: 4\n"
      "  left_shoulder: 3\n"
      "  right_shoulder: 3.5\n"
      "  elevation_bounds: [0, 5]\n"
      "  position_precision: 0.01\n"
      "  orientation_precision: 0.5\n"
      "  points:\n"
      "    start:\n"
      "      xypoint: [20, 30, -45]\n"
      "      zpoint: [1, 1, 0, 0]\n"
      "  connections:\n"
      "    s1:\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"points.start\"]\n"
      "      arc: [20, 180]\n"
      "      z_end: [\"ref\", [10, 0, 0, 0]]\n"
      "    s2:\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"connections.s1.end\"]\n"
      "      arc: [15.8, -180]\n"
      "      z_end: [\"ref\", [1, -1, 0, 0]]\n"
      "  groups: {}\n"};
  const std::string kMultilaneLineAndArcSegmentsYaml{
      "maliput_multilane_builder:\n"
      "  id: \"two_connected_segments\"\n"
      "  lane_width: 4\n"
      "  left_shoulder: 3\n"
      "  right_shoulder: 3.5\n"
      "  elevation_bounds: [0, 5]\n"
      "  position_precision: 0.01\n"
      "  orientation_precision: 0.5\n"
      "  points:\n"
      "    start:\n"
      "      xypoint: [20, 30, -45]\n"
      "      zpoint: [1, 1, 0, 0]\n"
      "  connections:\n"
      "    s1:\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"points.start\"]\n"
      "      length: 100\n"
      "      z_end: [\"ref\", [10, 0, 0, 0]]\n"
      "    s2:\n"
      "      lanes: [1, 0, 2.1]\n"
      "      start: [\"ref\", \"connections.s1.end\"]\n"
      "      arc: [20, 75]\n"
      "      z_end: [\"ref\", [1, -1, 0, 0]]\n"
      "  groups: {}\n"};
};

// Checks the metadata of different RoadGeometries with two segments of
// different geometry types.
TEST_F(MultilaneLoaderSingleLaneSegmentConnectionTest, MetadataChecks) {
  auto check_road_geometry_metadata = [&](const api::RoadGeometry& rg) {
    EXPECT_EQ(rg.id().string(), kRoadGeometryName);
    EXPECT_EQ(rg.linear_tolerance(), kLinearTolerance);
    EXPECT_NEAR(rg.angular_tolerance(), kAngularTolerance, kVeryExact);
    EXPECT_EQ(rg.num_branch_points(), kNumBranchpoints);
    EXPECT_EQ(rg.num_junctions(), kNumJunctions);
    EXPECT_EQ(rg.junction(0)->id().string(), kFirstJunctionName);
    EXPECT_EQ(rg.junction(0)->num_segments(), kNumSegments);
    EXPECT_EQ(rg.junction(0)->segment(0)->id().string(), kFirstSegmentName);
    EXPECT_EQ(rg.junction(0)->segment(0)->num_lanes(), kNumLanes);
    EXPECT_EQ(rg.junction(0)->segment(0)->lane(0)->id().string(),
              kFirstLaneName);
    EXPECT_EQ(rg.junction(1)->segment(0)->id().string(), kSecondSegmentName);
    EXPECT_EQ(rg.junction(1)->segment(0)->num_lanes(), kNumLanes);
    EXPECT_EQ(rg.junction(1)->segment(0)->lane(0)->id().string(),
              kSecondLaneName);

    const api::Lane* s1_lane = rg.junction(0)->segment(0)->lane(0);
    const api::Lane* s2_lane = rg.junction(1)->segment(0)->lane(0);
    EXPECT_NE(s1_lane, nullptr);
    EXPECT_NE(s2_lane, nullptr);
    EXPECT_EQ(s1_lane->GetBranchPoint(api::LaneEnd::kStart)->GetASide()->size(),
              1);
    EXPECT_EQ(
        s1_lane->GetBranchPoint(api::LaneEnd::kStart)->GetASide()->get(0).lane,
        s1_lane);
    EXPECT_EQ(s1_lane->GetBranchPoint(api::LaneEnd::kStart)->GetBSide()->size(),
              0);
    EXPECT_EQ(
        s1_lane->GetBranchPoint(api::LaneEnd::kFinish)->GetASide()->size(), 1);
    EXPECT_EQ(
        s1_lane->GetBranchPoint(api::LaneEnd::kFinish)->GetASide()->get(0).lane,
        s1_lane);
    EXPECT_EQ(
        s1_lane->GetBranchPoint(api::LaneEnd::kFinish)->GetBSide()->size(), 1);
    EXPECT_EQ(
        s1_lane->GetBranchPoint(api::LaneEnd::kFinish)->GetBSide()->get(0).lane,
        s2_lane);

    EXPECT_EQ(s2_lane->GetBranchPoint(api::LaneEnd::kStart)->GetASide()->size(),
              1);
    EXPECT_EQ(
        s2_lane->GetBranchPoint(api::LaneEnd::kStart)->GetASide()->get(0).lane,
        s1_lane);
    EXPECT_EQ(s2_lane->GetBranchPoint(api::LaneEnd::kStart)->GetBSide()->size(),
              1);
    EXPECT_EQ(
        s2_lane->GetBranchPoint(api::LaneEnd::kStart)->GetBSide()->get(0).lane,
        s2_lane);
    EXPECT_EQ(
        s2_lane->GetBranchPoint(api::LaneEnd::kFinish)->GetASide()->size(), 1);
    EXPECT_EQ(
        s2_lane->GetBranchPoint(api::LaneEnd::kFinish)->GetASide()->get(0).lane,
        s2_lane);
    EXPECT_EQ(
        s2_lane->GetBranchPoint(api::LaneEnd::kFinish)->GetBSide()->size(), 0);
  };

  std::unique_ptr<const api::RoadGeometry> line_dut =
      Load(kMultilaneLineSegmentsYaml);
  EXPECT_NE(line_dut, nullptr);
  check_road_geometry_metadata(*line_dut);

  std::unique_ptr<const api::RoadGeometry> arc_dut =
      Load(kMultilaneArcSegmentsYaml);
  EXPECT_NE(arc_dut, nullptr);
  check_road_geometry_metadata(*arc_dut);

  std::unique_ptr<const api::RoadGeometry> line_and_arc_dut =
      Load(kMultilaneLineAndArcSegmentsYaml);
  EXPECT_NE(line_and_arc_dut, nullptr);
  check_road_geometry_metadata(*line_and_arc_dut);
}

// Checks the construction of two consecutive single line-lane segments.
TEST_F(MultilaneLoaderSingleLaneSegmentConnectionTest,
       TwoSingleLaneLineSegments) {
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(kMultilaneLineSegmentsYaml);
  EXPECT_NE(rg, nullptr);

  const Lane* s1_lane =
      dynamic_cast<const Lane*>(rg->junction(0)->segment(0)->lane(0));

  // The following length value comes from computing the in Octave the result of
  // LineRoadCurve::s_from_p(1, 2.1). This is because, RoadCurve are not good at
  // computing the arc length of roads whose elevation polynomial order is other
  // than linear.
  const double s1_lane_length = 100.404183179786;
  EXPECT_NEAR(s1_lane->length(), s1_lane_length, kVeryExact);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s1_lane->ToGeoPosition({0., 0., 0.}),
      {21.4849242404918, 31.4849242404918, 1.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s1_lane->ToGeoPosition({s1_lane_length, 0., 0.}),
      {92.1956023591465, -39.2257538781630, 10.}, kVeryExact));

  const Lane* s2_lane =
      dynamic_cast<const Lane*>(rg->junction(1)->segment(0)->lane(0));

  const double s2_lane_length = 100.;
  EXPECT_NEAR(s2_lane->length(), s2_lane_length, kVeryExact);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s2_lane->ToGeoPosition({0., 0., 0.}),
      {92.1956023591465, -39.2257538781630, 10.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s2_lane->ToGeoPosition({s2_lane_length, 0., 0.}),
      {162.906280477801, -109.936431996818, 10.}, kVeryExact));
}

// Checks the construction of two consecutive single arc-lane segments.
TEST_F(MultilaneLoaderSingleLaneSegmentConnectionTest,
       TwoSingleLaneArcSegments) {
  std::unique_ptr<const api::RoadGeometry> rg = Load(kMultilaneArcSegmentsYaml);
  EXPECT_NE(rg, nullptr);

  const Lane* s1_lane =
      static_cast<const Lane*>(rg->junction(0)->segment(0)->lane(0));

  // The following length value comes from computing the in Octave the result of
  // ArcRoadCurve::s_from_p(1, 2.1). This is because, RoadCurve are not good at
  // computing the arc length of roads whose elevation polynomial order is other
  // than linear.
  const double s1_lane_length = 56.9501531705845;
  EXPECT_NEAR(s1_lane->length(), s1_lane_length, kVeryExact);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s1_lane->ToGeoPosition({0., 0., 0.}),
      {21.4849242404918, 31.4849242404918, 1.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s1_lane->ToGeoPosition({s1_lane_length, 0., 0.}),
      {46.7993470069702, 56.7993470069702, 10.}, kVeryExact));

  const Lane* s2_lane =
      static_cast<const Lane*>(rg->junction(1)->segment(0)->lane(0));

  const double s2_lane_length = 56.9501531705845;
  EXPECT_NEAR(s2_lane->length(), s2_lane_length, kVeryExact);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s2_lane->ToGeoPosition({0., 0., 0.}),
      {46.7993470069702, 56.7993470069702, 10.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s2_lane->ToGeoPosition({s2_lane_length, 0., 0.}),
      {72.1137697734486, 82.1137697734486, 1.}, kVeryExact));
}

// Checks the construction of two consecutive single lane segments. The first
// one is a line and the second one is an arc.
TEST_F(MultilaneLoaderSingleLaneSegmentConnectionTest, LineAndArcSegments) {
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(kMultilaneLineAndArcSegmentsYaml);
  EXPECT_NE(rg, nullptr);

  const Lane* s1_lane =
      static_cast<const Lane*>(rg->junction(0)->segment(0)->lane(0));

  // The following length value comes from computing the in Octave the result of
  // LineRoadCurve::s_from_p(1, 2.1). This is because, RoadCurve are not good at
  // computing the arc length of roads whose elevation polynomial order is other
  // than linear.
  const double s1_lane_length = 100.404183179786;
  EXPECT_NEAR(s1_lane->length(), s1_lane_length, kVeryExact);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s1_lane->ToGeoPosition({0., 0., 0.}),
      {21.4849242404918, 31.4849242404918, 1.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s1_lane->ToGeoPosition({s1_lane_length, 0., 0.}),
      {92.1956023591465, -39.2257538781630, 10.}, kVeryExact));

  const Lane* s2_lane =
      static_cast<const Lane*>(rg->junction(1)->segment(0)->lane(0));

  // The following length value comes from computing the in Octave the result of
  // ArcRoadCurve::s_from_p(1, 2.1). This is because, RoadCurve are not good at
  // computing the arc length of roads whose elevation polynomial order is other
  // than linear.
  const double s2_lane_length = 25.1000772815635;
  EXPECT_NEAR(s2_lane->length(), s2_lane_length, kVeryExact);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s2_lane->ToGeoPosition({0., 0., 0.}),
      {92.1956023591465, -39.2257538781630, 10.}, kVeryExact));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      s2_lane->ToGeoPosition({s2_lane_length, 0., 0.}),
      {113.8028137423857, -42.0703972226652, 1.}, kVeryExact));
}

struct JunctionInfo {
  std::string name{};
  int num_segments;
};

struct SegmentInfo {
  std::string name{};
  int num_lanes{};
};

struct LaneInfo {
  api::GeoPosition start{};
  api::GeoPosition end{};
  std::string left_lane_name;
  std::string right_lane_name;
};

struct BranchPointLaneIds {
  std::vector<std::string> start_a_side;
  std::vector<std::string> start_b_side;
  std::vector<std::string> finish_a_side;
  std::vector<std::string> finish_b_side;
};

// This test checks the creation of a RoadGeometry like the following:
//
// <pre>
//                             s3
//               *-0-----------<------------*
//              0                            *
//         s4  ∨                              ∧ s2
//              *              s1            0
//               *-2----------->------------*
//               *-1----------->------------*
//              **-0----------->------------**
//             **                            01
//         s7 ∧∧                              ∨∨  s5
//             10              s6            **
//              **-0-----------<------------**
//               *-1-----------<------------*
//
// </pre>
GTEST_TEST(MultilaneLoader, MultipleLaneSegmentCircuit) {
  const double kVeryExact{1e-12};
  const std::string kMultilaneYaml =
      "maliput_multilane_builder:\n"
      "  id: \"line_and_arc\"\n"
      "  lane_width: 5\n"
      "  left_shoulder: 1\n"
      "  right_shoulder: 1\n"
      "  elevation_bounds: [0, 5]\n"
      "  position_precision: 0.01\n"
      "  orientation_precision: 0.5\n"
      "  points:\n"
      "    a:\n"
      "      xypoint: [0, 0, 0]\n"
      "      zpoint: [0, 0, 0, 0]\n"
      "  connections:\n"
      "    s1:\n"
      "      lanes: [3, 0, 0]\n"
      "      start: [\"ref\", \"points.a\"]\n"
      "      length: 50\n"
      "      z_end: [\"ref\", [0, 0, 0, 0]]\n"
      "    s2:\n"
      "      lanes: [1, 0, 0]\n"
      "      start: [\"ref\", \"connections.s1.2.end\"]\n"
      "      arc: [10, 180]\n"
      "      z_end: [\"ref\", [0, 0, 0, 0]]\n"
      "    s3:\n"
      "      lanes: [1, 0, 0]\n"
      "      start: [\"ref\", \"connections.s2.end\"]\n"
      "      length: 50\n"
      "      z_end: [\"ref\", [0, 0, 0, 0]]\n"
      "    s4:\n"
      "      lanes: [1, 0, 0]\n"
      "      start: [\"ref\", \"connections.s3.end\"]\n"
      "      arc: [10, 180]\n"
      "      z_end: [\"ref\", [0, 0, 0, 0]]\n"
      "    s5:\n"
      "      lanes: [2, 0, 0]\n"
      "      start: [\"ref\", \"connections.s1.0.end\"]\n"
      "      arc: [20, -180]\n"
      "      z_end: [\"ref\", [0, 0, 0, 0]]\n"
      "    s6:\n"
      "      lanes: [2, 0, 0]\n"
      "      start: [\"ref\", \"connections.s5.0.end\"]\n"
      "      length: 50\n"
      "      z_end: [\"ref\", [0, 0, 0, 0]]\n"
      "    s7:\n"
      "      lanes: [2, 0, 0]\n"
      "      start: [\"ref\", \"connections.s6.0.end\"]\n"
      "      arc: [20, -180]\n"
      "      z_end: [\"ref\", [0, 0, 0, 0]]\n"
      "  groups: {}\n";

  const std::map<int, JunctionInfo> kJunctionTruthMap{
      {0, {"j:s1", 1}}, {1, {"j:s2", 1}}, {2, {"j:s3", 1}}, {3, {"j:s4", 1}},
      {4, {"j:s5", 1}}, {5, {"j:s6", 1}}, {6, {"j:s7", 1}}};

  const std::map<std::string, SegmentInfo> kSegmentTruthMap{
      {"j:s1", {"s:s1", 3}}, {"j:s2", {"s:s2", 1}}, {"j:s3", {"s:s3", 1}},
      {"j:s4", {"s:s4", 1}}, {"j:s5", {"s:s5", 2}}, {"j:s6", {"s:s6", 2}},
      {"j:s7", {"s:s7", 2}}};

  const std::map<std::string, LaneInfo> kLaneTruthMap{
      {"l:s1_0", {{0., 0., 0.}, {50., 0., 0.}, "l:s1_1", ""}},
      {"l:s1_1", {{0., 5., 0.}, {50., 5., 0.}, "l:s1_2", "l:s1_0"}},
      {"l:s1_2", {{0., 10., 0.}, {50., 10., 0.}, "", "l:s1_1"}},

      {"l:s2_0", {{50., 10., 0.}, {50., 30., 0.}, "", ""}},

      {"l:s3_0", {{50., 30., 0.}, {0., 30., 0.}, "", ""}},

      {"l:s4_0", {{0., 30., 0.}, {0., 10., 0.}, "", ""}},

      {"l:s5_0", {{50., 0., 0.}, {50., -40., 0.}, "l:s5_1", ""}},
      {"l:s5_1", {{50., 5., 0.}, {50., -45., 0.}, "", "l:s5_0"}},

      {"l:s6_0", {{50., -40., 0.}, {0., -40., 0.}, "l:s6_1", ""}},
      {"l:s6_1", {{50., -45., 0.}, {0., -45., 0.}, "", "l:s6_0"}},

      {"l:s7_0", {{0., -40., 0.}, {0., 0., 0.}, "l:s7_1", ""}},
      {"l:s7_1", {{0., -45., 0.}, {0., 5., 0.}, "", "l:s7_0"}}};

  const std::map<std::string, BranchPointLaneIds> kLaneBranchPointTruthMap{
      {"l:s1_0", {{"l:s1_0"}, {"l:s7_0"}, {"l:s1_0"}, {"l:s5_0"}}},
      {"l:s1_1", {{"l:s1_1"}, {"l:s7_1"}, {"l:s1_1"}, {"l:s5_1"}}},
      {"l:s1_2", {{"l:s1_2"}, {"l:s4_0"}, {"l:s1_2"}, {"l:s2_0"}}},

      {"l:s2_0", {{"l:s1_2"}, {"l:s2_0"}, {"l:s2_0"}, {"l:s3_0"}}},

      {"l:s3_0", {{"l:s2_0"}, {"l:s3_0"}, {"l:s3_0"}, {"l:s4_0"}}},

      {"l:s4_0", {{"l:s3_0"}, {"l:s4_0"}, {"l:s1_2"}, {"l:s4_0"}}},

      {"l:s5_0", {{"l:s1_0"}, {"l:s5_0"}, {"l:s5_0"}, {"l:s6_0"}}},
      {"l:s5_1", {{"l:s1_1"}, {"l:s5_1"}, {"l:s5_1"}, {"l:s6_1"}}},

      {"l:s6_0", {{"l:s5_0"}, {"l:s6_0"}, {"l:s6_0"}, {"l:s7_0"}}},
      {"l:s6_1", {{"l:s5_1"}, {"l:s6_1"}, {"l:s6_1"}, {"l:s7_1"}}},

      {"l:s7_0", {{"l:s6_0"}, {"l:s7_0"}, {"l:s1_0"}, {"l:s7_0"}}},
      {"l:s7_1", {{"l:s6_1"}, {"l:s7_1"}, {"l:s1_1"}, {"l:s7_1"}}},
  };

  std::unique_ptr<const api::RoadGeometry> rg = Load(kMultilaneYaml);
  EXPECT_NE(rg, nullptr);
  EXPECT_EQ(rg->num_junctions(), kJunctionTruthMap.size());
  EXPECT_EQ(rg->num_branch_points(), 12);
  for (const auto& junction_info : kJunctionTruthMap) {
    // Checks junction properties.
    EXPECT_EQ(rg->junction(junction_info.first)->id().string(),
              junction_info.second.name);
    EXPECT_EQ(rg->junction(junction_info.first)->num_segments(),
              junction_info.second.num_segments);
    // Checks segment properties.
    const SegmentInfo& segment_info =
        kSegmentTruthMap.at(junction_info.second.name);
    EXPECT_EQ(rg->junction(junction_info.first)->segment(0)->id().string(),
              segment_info.name);
    EXPECT_EQ(rg->junction(junction_info.first)->segment(0)->num_lanes(),
              segment_info.num_lanes);
    // Checks lane properties.
    const api::Segment* segment = rg->junction(junction_info.first)->segment(0);
    for (int i = 0; i < segment->num_lanes(); ++i) {
      const api::Lane* lane = segment->lane(i);
      EXPECT_NE(kLaneTruthMap.find(lane->id().string()), kLaneTruthMap.end());
      const LaneInfo& lane_info = kLaneTruthMap.at(lane->id().string());
      EXPECT_TRUE(api::test::IsGeoPositionClose(
          lane->ToGeoPosition({0., 0., 0.}), lane_info.start, kVeryExact));
      EXPECT_TRUE(api::test::IsGeoPositionClose(
          lane->ToGeoPosition({lane->length(), 0., 0.}), lane_info.end,
          kVeryExact));
      if (lane_info.left_lane_name.empty()) {
        EXPECT_EQ(lane->to_left(), nullptr);
      } else {
        EXPECT_EQ(lane->to_left()->id().string(), lane_info.left_lane_name);
      }
      if (lane_info.right_lane_name.empty()) {
        EXPECT_EQ(lane->to_right(), nullptr);
      } else {
        EXPECT_EQ(lane->to_right()->id().string(), lane_info.right_lane_name);
      }
      // Checks the BranchPoints of the Lane.
      const BranchPointLaneIds& bp_lane_info =
          kLaneBranchPointTruthMap.at(lane->id().string());
      const api::BranchPoint* const start_bp =
          lane->GetBranchPoint(api::LaneEnd::kStart);
      EXPECT_EQ(start_bp->GetASide()->size(), bp_lane_info.start_a_side.size());
      for (int lane_index = 0; lane_index < start_bp->GetASide()->size();
           lane_index++) {
        EXPECT_EQ(start_bp->GetASide()->get(lane_index).lane->id().string(),
                  bp_lane_info.start_a_side[lane_index]);
      }
      EXPECT_EQ(start_bp->GetBSide()->size(), bp_lane_info.start_b_side.size());
      for (int lane_index = 0; lane_index < start_bp->GetBSide()->size();
           lane_index++) {
        EXPECT_EQ(start_bp->GetBSide()->get(lane_index).lane->id().string(),
                  bp_lane_info.start_b_side[lane_index]);
      }
      const api::BranchPoint* const end_bp =
          lane->GetBranchPoint(api::LaneEnd::kFinish);
      EXPECT_EQ(end_bp->GetASide()->size(), bp_lane_info.finish_a_side.size());
      for (int lane_index = 0; lane_index < end_bp->GetASide()->size();
           lane_index++) {
        EXPECT_EQ(end_bp->GetASide()->get(lane_index).lane->id().string(),
                  bp_lane_info.finish_a_side[lane_index]);
      }
      EXPECT_EQ(end_bp->GetBSide()->size(), bp_lane_info.finish_b_side.size());
      for (int lane_index = 0; lane_index < end_bp->GetBSide()->size();
           lane_index++) {
        EXPECT_EQ(end_bp->GetBSide()->get(lane_index).lane->id().string(),
                  bp_lane_info.finish_b_side[lane_index]);
      }
    }
  }
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
