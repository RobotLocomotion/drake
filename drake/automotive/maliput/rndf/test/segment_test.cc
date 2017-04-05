#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-12;
const double kAngularTolerance = 1e-12;

// It tests some segment creation checks and the values of the the lanes indexes
GTEST_TEST(RNDFSegmentTest, MetadataLane) {
  RoadGeometry rg({"BareSegment"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});

  EXPECT_EQ(s1->num_lanes(), 0);

  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  s1->NewSplineLane({"l1"}, control_points, 5.);

  EXPECT_EQ(s1->num_lanes(), 1);

  EXPECT_NO_THROW(
    s1->NewSplineLane({"l2"}, control_points, 5.));

  EXPECT_EQ(s1->num_lanes(), 2);
}


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
