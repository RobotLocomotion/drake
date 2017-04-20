#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace api {
namespace {

// Tests the streaming string operators for the following Maliput abstractions:
//   - JunctionId
//   - LaneId
//   - RoadGeometryId
//   - SegmentId
GTEST_TEST(MaliputApiTest, TestIdToStringStream) {
  std::stringstream buffer;

  // Tests JunctionId.
  const std::string junction_name = "foo junction id";
  buffer << JunctionId({junction_name});
  EXPECT_EQ(buffer.str(), "Junction(" + junction_name + ")");
  buffer.str("");
  buffer.clear();

  // Tests LaneId.
  const std::string lane_name = "foo lane id";
  buffer << LaneId({lane_name});
  EXPECT_EQ(buffer.str(), "Lane(" + lane_name + ")");
  buffer.str("");
  buffer.clear();

  // Tests RoadGeometryId.
  const std::string road_geometry_name = "foo road id";
  buffer << RoadGeometryId({road_geometry_name});
  EXPECT_EQ(buffer.str(), "RoadGeometry(" + road_geometry_name + ")");
  buffer.str("");
  buffer.clear();

  // Tests SegmentId.
  const std::string segment_name = "foo segment id";
  buffer << SegmentId({segment_name});
  EXPECT_EQ(buffer.str(), "Segment(" + segment_name + ")");
}

// Tests the streaming string operators for the following Maliput abstractions:
//   - GeoPosition
//   - LaneEnd::Which
//   - LanePosition
//   - Rotation
GTEST_TEST(MaliputApiTest, TestLaneDataToStringStream) {
  std::stringstream buffer;

  // Tests GeoPosition.
  buffer << GeoPosition(1.5, 2.2, 3.7);
  EXPECT_EQ(buffer.str(), "(x = 1.5, y = 2.2, z = 3.7)");
  buffer.str("");
  buffer.clear();

  // Tests LaneEnd::Which.
  LaneEnd::Which start = LaneEnd::kStart;
  buffer <<  start;
  EXPECT_EQ(buffer.str(), "start");
  buffer.str("");
  buffer.clear();
  LaneEnd::Which end = LaneEnd::kFinish;
  buffer << end;
  EXPECT_EQ(buffer.str(), "finish");
  buffer.str("");
  buffer.clear();

  // Tests LanePosition.
  buffer << LanePosition(0.2, 9.1, 15.2);
  EXPECT_EQ(buffer.str(), "(s = 0.2, r = 9.1, h = 15.2)");
  buffer.str("");
  buffer.clear();

  // Tests Rotation.
  buffer << Rotation({1, 2, 3});
  EXPECT_EQ(buffer.str(), "(roll = 1, pitch = 2, yaw = 3)");
  buffer.str("");
  buffer.clear();
}

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
