#include "drake/automotive/maliput/rndf/segment.h"

#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"

namespace drake {
namespace maliput {
namespace rndf {
namespace {

// The following tolerances are very strict as they are not used to compute
// anything in the following tests. However, we need them for the RoadGeometry
// constuctor.
const double kLinearTolerance = 1e-12;
const double kAngularTolerance = 1e-12;

// The following tests segment creation and the values of the lanes' indexes.
GTEST_TEST(RNDFSegmentTest, MetadataLane) {
  RoadGeometry rg(api::RoadGeometryId{"BareSegment"},
                  kLinearTolerance, kAngularTolerance);
  Segment* s1 =
      rg.NewJunction(api::JunctionId{"j1"})->NewSegment(api::SegmentId{"s1"});

  EXPECT_EQ(s1->junction(), rg.junction(0));
  EXPECT_EQ(s1->num_lanes(), 0);

  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  // As I add only one control_point, I expect the creator to throw.
  EXPECT_THROW(s1->NewSplineLane(api::LaneId{"l1"}, control_points, 5.),
               std::runtime_error);
  // Now I add the second control_point and expect it to not throw.
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  EXPECT_NO_THROW(s1->NewSplineLane(api::LaneId{"l1"}, control_points, 5.));
  EXPECT_EQ(s1->num_lanes(), 1);

  EXPECT_NO_THROW(s1->NewSplineLane(api::LaneId{"l2"}, control_points, 5.));

  EXPECT_EQ(s1->num_lanes(), 2);

  EXPECT_THROW(s1->lane(3), std::runtime_error);
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
