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
namespace {

// TODO(agalbachicar)    Missing tests for non-zero EndpointZ and multi-lane
//                       segments.

// TODO(agalbachicar)    Missing tests for ".reverse" semantic in "start",
//                       "explicit_end" and "z_end".

// Checks that the minimal YAML passes with an empty RoadGeometry.
GTEST_TEST(MultilaneLoaderTest, MinimalCorrectYaml) {
  const double kVeryExact{1e-12};
  const char* kMultilaneYaml = R"R(maliput_multilane_builder:
  id: "empty_road_geometry"
  lane_width: 4
  left_shoulder: 1
  right_shoulder: 2
  elevation_bounds: [0, 5]
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  points: {}
  connections: {}
  groups: {}
)R";

  std::unique_ptr<const api::RoadGeometry> rg =
      Load(std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
  EXPECT_EQ(rg->id().string(), "empty_road_geometry");
  EXPECT_EQ(rg->linear_tolerance(), 0.01);
  EXPECT_NEAR(rg->angular_tolerance(), 0.5 * M_PI / 180., kVeryExact);
  EXPECT_EQ(rg->num_junctions(), 0);
  EXPECT_EQ(rg->num_branch_points(), 0);
}

// Structures for later test checks.
struct JunctionInfo {
  std::string name{};
  int num_segments{};
};

struct SegmentInfo {
  std::string name{};
  int num_lanes{};
};

struct LaneInfo {
  api::GeoPosition start{};
  api::GeoPosition end{};
  double length{};
  double r0{};
  std::string left_lane_name;
  std::string right_lane_name;
};

struct BranchPointLaneIds {
  std::vector<std::string> start_a_side;
  std::vector<std::string> start_b_side;
  std::vector<std::string> finish_a_side;
  std::vector<std::string> finish_b_side;
};

// These tests exercise the following RoadGeometry:
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
//             **   |  s13                   01
//         s7 ∧∧    ∧                         ∨∨  s5
//             10   |  s12     s6            **
//              **-0|----------<------------**
//               *-1|----------<------------*
//         s8   0  0|   s11
//             ∨    ∧
//         s9   0  0   s10
//               **
// </pre>
//
// Given that the graph is 2D, it must be explained segment elevation profiles.
// Segments from s1 to s8 are all flat and on the ground. s9 goes up from z=0 to
// z=10m and s10 travels parallel to the ground at z=10m. s11 goes down again to
// reach z=0. s12 is flat and elevated too and goes over s6 at 10 meters. s13
// goes down and hits the ground at the end Endpoint in between s6 and s1.
class MultilaneLoaderMultipleSegmentCircuitTest : public ::testing::Test {
 protected:
  const std::string kMultilaneYaml = R"R(maliput_multilane_builder:
  id: "line_and_arc"
  lane_width: 5
  left_shoulder: 1
  right_shoulder: 1
  elevation_bounds: [0, 5]
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  points:
    a:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
  connections:
    s1:
      lanes: [3, 0, 0]
      start: ["ref", "points.a.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0, 0]]
    s2:
      lanes: [1, 0, 5]
      start: ["ref", "connections.s1.end.1.forward"]
      arc: [15, 180]
      z_end: ["ref", [0, 0, 0, 0]]
    s3:
      lanes: [1, 0, 5]
      start: ["ref", "connections.s2.end.ref.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0, 0]]
    s4:
      lanes: [1, 0, 5]
      start: ["ref", "connections.s3.end.ref.forward"]
      arc: [15, 180]
      explicit_end: ["ref", "connections.s1.start.ref.forward"]
    s5:
      lanes: [2, 0, 0]
      start: ["ref", "connections.s1.end.0.forward"]
      arc: [20, -180]
      z_end: ["ref", [0, 0, 0, 0]]
    s6:
      lanes: [2, 0, 0]
      start: ["ref", "connections.s5.end.0.forward"]
      length: 50
      z_end: ["ref", [0, 0, 0, 0]]
    s7:
      lanes: [2, 0, 0]
      start: ["ref", "connections.s6.end.0.forward"]
      arc: [20, -180]
      explicit_end: ["ref", "connections.s1.start.ref.forward"]
    s8:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s6.end.1.forward"]
      arc: [20, 90]
      z_end: ["ref", [0, 0, 0, 0]]
    s9:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s8.end.ref.forward"]
      arc: [20, 90]
      z_end: ["ref", [10, 0, 0, 0]]
    s10:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s9.end.ref.forward"]
      arc: [20, 90]
      explicit_end: ["ref", "connections.s9.end.ref.forward"]
    s11:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s10.end.ref.forward"]
      arc: [20, 90]
      explicit_end: ["ref", "connections.s6.end.1.forward"]
    s12:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s10.end.0.forward"]
      length: 30
      explicit_end: ["ref", "connections.s10.end.ref.forward"]
    s13:
      lanes: [1, 0, 0]
      start: ["ref", "connections.s12.end.0.forward"]
      length: 15
      z_end: ["ref", [0, 0, 0, 0]]
  groups: {}
)R";
  const std::map<int, JunctionInfo> kJunctionTruthMap{
      {0, {"j:s1", 1}}, {1, {"j:s2", 1}}, {2, {"j:s3", 1}}, {3, {"j:s4", 1}},
      {4, {"j:s5", 1}}, {5, {"j:s6", 1}}, {6, {"j:s7", 1}}, {7, {"j:s8", 1}},
      {8, {"j:s9", 1}}, {9, {"j:s10", 1}}, {10, {"j:s11", 1}},
      {11, {"j:s12", 1}}, {12, {"j:s13", 1}}};

  const std::map<std::string, SegmentInfo> kSegmentTruthMap{
      {"j:s1", {"s:s1", 3}}, {"j:s2", {"s:s2", 1}}, {"j:s3", {"s:s3", 1}},
      {"j:s4", {"s:s4", 1}}, {"j:s5", {"s:s5", 2}}, {"j:s6", {"s:s6", 2}},
      {"j:s7", {"s:s7", 2}}, {"j:s8", {"s:s8", 1}}, {"j:s9", {"s:s9", 1}},
      {"j:s10", {"s:s10", 1}}, {"j:s11", {"s:s11", 1}}, {"j:s12", {"s:s12", 1}},
      {"j:s13", {"s:s13", 1}}};

  const std::map<std::string, LaneInfo> kLaneTruthMap{
      {"l:s1_0", {{0., 0., 0.}, {50., 0., 0.}, 50., 0., "l:s1_1", ""}},
      {"l:s1_1", {{0., 5., 0.}, {50., 5., 0.}, 50., 5., "l:s1_2", "l:s1_0"}},
      {"l:s1_2", {{0., 10., 0.}, {50., 10., 0.}, 50., 10., "",  "l:s1_1"}},

      {"l:s2_0", {{50., 10., 0.}, {50., 30., 0.}, M_PI * 10., 5., "", ""}},

      {"l:s3_0", {{50., 30., 0.}, {0., 30., 0.}, 50., 5., "", ""}},

      {"l:s4_0", {{0., 30., 0.}, {0., 10., 0.}, M_PI * 10., 5., "", ""}},

      {"l:s5_0", {{50., 0., 0.}, {50., -40., 0.}, M_PI * 20., 0., "l:s5_1",
                  ""}},
      {"l:s5_1", {{50., 5., 0.}, {50., -45., 0.}, M_PI * 25., 5., "",
                  "l:s5_0"}},

      {"l:s6_0", {{50., -40., 0.}, {0., -40., 0.}, 50., 0., "l:s6_1", ""}},
      {"l:s6_1", {{50., -45., 0.}, {0., -45., 0.}, 50., 5., "", "l:s6_0"}},

      {"l:s7_0", {{0., -40., 0.}, {0., 0., 0.}, M_PI * 20., 0., "l:s7_1", ""}},
      {"l:s7_1", {{0., -45., 0.}, {0., 5., 0.}, M_PI * 25., 5., "", "l:s7_0"}},

      {"l:s8_0", {{0., -45., 0.}, {-20., -65., 0.}, M_PI / 2. * 20., 0., "",
                  ""}},

      {"l:s9_0", {{-20., -65., 0.}, {0., -85., 10.}, 32.9690830947562, 0., "",
                  ""}},

      {"l:s10_0", {{0., -85., 10.}, {20., -65., 10.}, M_PI / 2. * 20., 0., "",
                   ""}},

      {"l:s11_0", {{20., -65., 10.}, {0., -45., 0.}, 32.9690830947562, 0., "",
                   ""}},

      {"l:s12_0", {{20., -65., 10.}, {20., -35., 10.}, 30., 0., "", ""}},

      {"l:s13_0", {{20., -35., 10.}, {20., -20., 0.}, 18.0277563773199, 0., "",
                   ""}}};

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
      {"l:s6_1", {{"l:s5_1"}, {"l:s6_1"}, {"l:s6_1", "l:s11_0"},
                  {"l:s7_1", "l:s8_0"}}},

      {"l:s7_0", {{"l:s6_0"}, {"l:s7_0"}, {"l:s1_0"}, {"l:s7_0"}}},
      {"l:s7_1", {{"l:s6_1", "l:s11_0"}, {"l:s7_1", "l:s8_0"}, {"l:s1_1"},
                  {"l:s7_1"}}},

      {"l:s8_0", {{"l:s6_1", "l:s11_0"}, {"l:s7_1", "l:s8_0"}, {"l:s8_0"},
                  {"l:s9_0"}}},

      {"l:s9_0", {{"l:s8_0"}, {"l:s9_0"}, {"l:s9_0"}, {"l:s10_0"}}},

      {"l:s10_0", {{"l:s9_0"}, {"l:s10_0"}, {"l:s10_0"},
                   {"l:s11_0", "l:s12_0"}}},

      {"l:s11_0", {{"l:s10_0"}, {"l:s11_0", "l:s12_0"}, {"l:s6_1", "l:s11_0"},
                   {"l:s7_1", "l:s8_0"}}},

      {"l:s12_0", {{"l:s10_0"}, {"l:s11_0", "l:s12_0"}, {"l:s12_0"},
                   {"l:s13_0"}}},

      {"l:s13_0", {{"l:s12_0"}, {"l:s13_0"}, {"l:s13_0"}, {}}}
  };
};

// Checks how the RoadGeometry is connected.
TEST_F(MultilaneLoaderMultipleSegmentCircuitTest, RoadStructure) {
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
  EXPECT_EQ(rg->num_junctions(), kJunctionTruthMap.size());
  EXPECT_EQ(rg->num_branch_points(), 17);
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

// Checks lane geometry by inspecting lane start and end GeoPositions, length
// and r0.
TEST_F(MultilaneLoaderMultipleSegmentCircuitTest, LaneGeometry) {
  const double kVeryExact{1e-12};
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);
  for (const auto& junction_info : kJunctionTruthMap) {
    // Checks lane properties.
    const api::Segment* segment = rg->junction(junction_info.first)->segment(0);
    for (int i = 0; i < segment->num_lanes(); ++i) {
      const Lane* lane = dynamic_cast<const Lane*>(segment->lane(i));
      EXPECT_NE(lane, nullptr);
      EXPECT_NE(kLaneTruthMap.find(lane->id().string()), kLaneTruthMap.end());
      const LaneInfo& lane_info = kLaneTruthMap.at(lane->id().string());
      EXPECT_TRUE(api::test::IsGeoPositionClose(
          lane->ToGeoPosition({0., 0., 0.}), lane_info.start, kVeryExact));
      EXPECT_TRUE(api::test::IsGeoPositionClose(
          lane->ToGeoPosition({lane->length(), 0., 0.}), lane_info.end,
          kVeryExact));
      EXPECT_NEAR(lane->length(), lane_info.length, kVeryExact);
      EXPECT_NEAR(lane->r0(), lane_info.r0, kVeryExact);
    }
  }
}

// TODO(agalbachicar)  Missing tests with non-zero superelevation segments.
TEST_F(MultilaneLoaderMultipleSegmentCircuitTest, LaneElevationPolynomials) {
  const double kVeryExact{1e-12};
  std::unique_ptr<const api::RoadGeometry> rg =
      Load(std::string(kMultilaneYaml));
  EXPECT_NE(rg, nullptr);

  // Finds a lane in a RoadGeometry by its id.
  auto find_lane = [&rg](const std::string& id) {
    for (int i = 0; i < rg->num_junctions(); ++i) {
      const api::Junction* junction = rg->junction(i);
      for (int j = 0; j < junction->num_segments(); ++j) {
        const api::Segment* segment = junction->segment(j);
        for (int k = 0; k < segment->num_lanes(); ++k) {
          if (segment->lane(k)->id().string() == id) {
            return segment->lane(k);
          }
        }
      }
    }
    return static_cast<const api::Lane*>(nullptr);
  };

  // Checks zero elevated linear lane elevation polynomial.
  const multilane::Lane* flat_planar_linear_dut =
      dynamic_cast<const multilane::Lane*>(find_lane("l:s1_1"));
  EXPECT_NE(flat_planar_linear_dut, nullptr);
  EXPECT_NEAR(flat_planar_linear_dut->elevation().a(), 0., kVeryExact);
  EXPECT_NEAR(flat_planar_linear_dut->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(flat_planar_linear_dut->elevation().c(), 0., kVeryExact);
  EXPECT_NEAR(flat_planar_linear_dut->elevation().d(), 0., kVeryExact);

  // Checks zero elevated arc lane elevation polynomial.
  const multilane::Lane* flat_planar_arc_dut =
      dynamic_cast<const multilane::Lane*>(find_lane("l:s5_1"));
  EXPECT_NE(flat_planar_arc_dut, nullptr);
  EXPECT_NEAR(flat_planar_arc_dut->elevation().a(), 0., kVeryExact);
  EXPECT_NEAR(flat_planar_arc_dut->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(flat_planar_arc_dut->elevation().c(), 0., kVeryExact);
  EXPECT_NEAR(flat_planar_arc_dut->elevation().d(), 0., kVeryExact);

  // Checks elevated planar linear lane elevation polynomial.
  const multilane::Lane* elevated_planar_linear_dut =
      dynamic_cast<const multilane::Lane*>(find_lane("l:s12_0"));
  EXPECT_NE(elevated_planar_linear_dut, nullptr);
  EXPECT_NEAR(elevated_planar_linear_dut->elevation().a(), 1. / 3., kVeryExact);
  EXPECT_NEAR(elevated_planar_linear_dut->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(elevated_planar_linear_dut->elevation().c(), 0., kVeryExact);
  EXPECT_NEAR(elevated_planar_linear_dut->elevation().d(), 0., kVeryExact);

  // Checks elevated planar arc lane elevation polynomial.
  const multilane::Lane* elevated_planar_arc_dut =
      dynamic_cast<const multilane::Lane*>(find_lane("l:s10_0"));
  EXPECT_NE(elevated_planar_arc_dut, nullptr);
  EXPECT_NEAR(
      elevated_planar_arc_dut->elevation().a(), 0.318309886183791, kVeryExact);
  EXPECT_NEAR(elevated_planar_arc_dut->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(elevated_planar_arc_dut->elevation().c(), 0., kVeryExact);
  EXPECT_NEAR(elevated_planar_arc_dut->elevation().d(), 0., kVeryExact);

  // Checks elevated planar linear lane elevation polynomial.
  const multilane::Lane* complex_linear_dut =
      dynamic_cast<const multilane::Lane*>(find_lane("l:s13_0"));
  EXPECT_NE(complex_linear_dut, nullptr);
  EXPECT_NEAR(
      complex_linear_dut->elevation().a(), 0.666666666666667, kVeryExact);
  EXPECT_NEAR(complex_linear_dut->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(complex_linear_dut->elevation().c(), -2., kVeryExact);
  EXPECT_NEAR(
      complex_linear_dut->elevation().d(), 1.33333333333333, kVeryExact);

  // Checks elevated planar arc lane elevation polynomial.
  const multilane::Lane* complex_arc_dut =
      dynamic_cast<const multilane::Lane*>(find_lane("l:s11_0"));
  EXPECT_NE(complex_arc_dut, nullptr);
  EXPECT_NEAR(complex_arc_dut->elevation().a(), 0.318309886183791, kVeryExact);
  EXPECT_NEAR(complex_arc_dut->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(complex_arc_dut->elevation().c(), -0.954929658551372, kVeryExact);
  EXPECT_NEAR(complex_arc_dut->elevation().d(), 0.636619772367581, kVeryExact);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
