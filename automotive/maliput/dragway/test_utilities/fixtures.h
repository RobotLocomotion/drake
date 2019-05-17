#pragma once

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"

namespace drake {
namespace maliput {
namespace dragway {

/// A fixture for tests that use a dragway.
class DragwayBasedTest : public ::testing::Test {
 protected:
  DragwayBasedTest();
  void SetUp() override;

  // The number of lanes was intentionally chosen to evaluate all combinations
  // of adjacent lanes, i.e., it includes a lane with just a lane to the left, a
  // lane with just a lane to the right, and a lane with lanes to both the left
  // and right. The rest of the parameters were arbitrarily chosen.
  const int kNumLanes{3};
  const double kLength{100};
  const double kLaneWidth{6};
  const double kShoulderWidth{1};
  const double kMaxHeight{5};
  const RoadGeometry dragway_;
  const api::Lane* right_lane_;
  const api::Lane* center_lane_;
  const api::Lane* left_lane_;
};


}  // namespace dragway
}  // namespace maliput
}  // namespace drake
