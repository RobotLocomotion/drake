#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"

namespace drake {
namespace maliput {
namespace rndf {

// The following tolerances are very strict as they are not used to compute
// anything in the following tests. However, we need them for the RoadGeometry
// constuctor.
const double kLinearTolerance = 1e-12;
const double kAngularTolerance = 1e-12;

// The following tests segment creation and the values of the lanes' indexes.
GTEST_TEST(RNDFSegmentTest, MetadataLane) {
  RoadGeometry rg({"BareSegment"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});

  EXPECT_EQ(s1->junction(), rg.junction(0));
  EXPECT_EQ(s1->num_lanes(), 0);

  s1->NewSplineLane({"l1"}, 5.);

  EXPECT_EQ(s1->num_lanes(), 1);

  EXPECT_NO_THROW(s1->NewSplineLane({"l2"}, 5.));

  EXPECT_EQ(s1->num_lanes(), 2);

  EXPECT_THROW(s1->lane(3), std::runtime_error);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
