#include "drake/automotive/maliput/utility/segment_analysis.h"

#include <memory>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/multilane/loader.h"

namespace drake {
namespace maliput {
namespace utility {
namespace {


GTEST_TEST(SegmentAnalysisAnalyzeConfluentSegments, BasicOperation) {
  // ---------------------------------------------------
  // |                                                 |
  // |                   ^^                            |
  // |                   || north2                     |
  // |                   ||                            |
  // |                   ||                            |
  // |                   oo                            |
  // |                   ^^                            |
  // |                   || north1                     |
  // |     east0         ||       east2                |
  // |    o--------->o---++---->o--------->            |
  // |    o--------->o---++---->o--------->o--\        |
  // |                \  || east1              \       |
  // |            turn \ ||                    | loop  |
  // |                  \oo                    |       |
  // |                   ^^                    |       |
  // |                   || north0             |       |
  // |                   ||                    |       |
  // |                   ||                    |       |
  // |                   oo                   /        |
  // |                   ^                   /         |
  // |                   |                  /          |
  // |                    \                /           |
  // |                     \--------------/            |
  // |                                                 |
  // ---------------------------------------------------
  const std::string dut_yaml = R"R(# -*- yaml -*-
---
# distances are meters; angles are degrees.
maliput_multilane_builder:
  id: dut
  lane_width: 1
  left_shoulder: 1
  right_shoulder: 1
  elevation_bounds: [0, 5]
  scale_length: 1.0
  linear_tolerance: 0.01
  angular_tolerance: 0.5
  computation_policy: prefer-speed
  points:
    origin:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
  connections:
    east0:
      lanes: [2, 0, 0]
      start: ["ref", "points.origin.forward"]
      length:  100
      z_end: ["ref", [0, 0, 0]]
    east1:
      lanes: [2, 0, 0]
      start: ["lane.0", "connections.east0.end.0.forward"]
      length: 100
      z_end: ["lane.0", [0, 0, 0]]
    east2:
      lanes: [2, 0, 0]
      start: ["lane.0", "connections.east1.end.0.forward"]
      length: 100
      z_end: ["lane.0", [0, 0, 0]]

    loop:
      lanes: [1, 0, 0]
      start: ["lane.0", "connections.east2.end.0.forward"]
      arc: [150, -270]
      z_end: ["lane.0", [0, 0, 0]]

    north0:
      lanes: [2, 0, 0]
      start: ["lane.0", "connections.loop.end.0.forward"]
      length: 100
      z_end: ["lane.0", [0, 0, 0]]
    north1:
      lanes: [2, 0, 0]
      start: ["lane.0", "connections.north0.end.0.forward"]
      length: 100
      z_end: ["lane.0", [0, 0, 0]]
    north2:
      lanes: [2, 0, 0]
      start: ["lane.0", "connections.north1.end.0.forward"]
      length: 100
      z_end: ["lane.0", [0, 0, 0]]

    turn:
      lanes: [1, 0, 0]
      start: ["lane.0", "connections.east0.end.0.forward"]
      arc: [50, -90]
      explicit_end: ["lane.0", "connections.north0.end.0.reverse"]

  groups: {}
)R";
  const std::unique_ptr<const api::RoadGeometry> rg =
      multilane::Load(multilane::BuilderFactory(), dut_yaml);

  const std::vector<std::unordered_set<const api::Segment*>> groups =
      AnalyzeConfluentSegments(rg.get());

  const auto lookup_segment = [&rg] (const std::string& id_string) {
    return rg->ById().GetSegment(api::SegmentId(id_string));
  };
  const std::set<std::set<const api::Segment*>> expected
      {{lookup_segment("s:east0")},
       {lookup_segment("s:east2")},
       {lookup_segment("s:north0")},
       {lookup_segment("s:north2")},
       {lookup_segment("s:loop")},
       {lookup_segment("s:north1"),
        lookup_segment("s:east1"),
        lookup_segment("s:turn")}
      };

  // Recast groups as "set of sets" (instead of "vector of unordered_sets")
  // to make equality testing trivial.
  std::set<std::set<const api::Segment*>> actual;
  for (const auto& group : groups) {
    std::set<const api::Segment*> set(group.begin(), group.end());
    actual.insert(set);
  }
  EXPECT_EQ(actual, expected);
}


}  // anonymous namespace
}  // namespace utility
}  // namespace maliput
}  // namespace drake
