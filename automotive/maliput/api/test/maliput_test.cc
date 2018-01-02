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
  buffer << Rotation::FromRpy(M_PI / 5., M_PI / 6., M_PI / 7.);
  EXPECT_EQ(buffer.str(),
            "(roll = 0.628319, pitch = 0.523599, yaw = 0.448799)");
  buffer.str("");
  buffer.clear();
}

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
