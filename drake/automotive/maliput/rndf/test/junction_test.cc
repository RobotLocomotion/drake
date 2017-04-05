#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/branch_point.h"
#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-12;
const double kAngularTolerance = 1e-12;

GTEST_TEST(RNDFJunctionTest, GettersTest) {
  RoadGeometry rg({"RG-GettersTest"}, kLinearTolerance, kAngularTolerance);
  Junction* junction = rg.NewJunction({"j:1"});

  EXPECT_EQ(junction->road_geometry(), &rg);
  EXPECT_EQ(junction->id().id, std::string("j:1"));
}

GTEST_TEST(RNDFJunctionTest, SegmentTest) {
  RoadGeometry rg({"RG-SegmentTest"}, kLinearTolerance, kAngularTolerance);
  Junction* junction = rg.NewJunction({"j:1"});

  EXPECT_EQ(junction->num_segments(), 0);

  Segment* s1 = junction->NewSegment({"s:1"});
  EXPECT_EQ(junction->num_segments(), 1);
  EXPECT_EQ(junction->segment(0), s1);

  Segment* s2 = junction->NewSegment({"s:2"});
  EXPECT_EQ(junction->num_segments(), 2);
  EXPECT_EQ(junction->segment(1), s2);

  EXPECT_THROW(junction->segment(-1), std::runtime_error);
  EXPECT_THROW(junction->segment(2), std::runtime_error);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
