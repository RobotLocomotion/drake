#include "drake/automotive/maliput/rndf/junction.h"

#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"

namespace drake {
namespace maliput {
namespace rndf {
namespace {

// The following tolerances are very strict as they are not used to compute
// anything in the following tests. However, we need them for the RoadGeometry
// constuctor.
const double kLinearTolerance = 1e-12;
const double kAngularTolerance = 1e-12;

// The following tests Junction creation and id assignment.
GTEST_TEST(RNDFJunctionTest, GettersTest) {
  RoadGeometry rg(api::RoadGeometryId{"RG-GettersTest"},
                  kLinearTolerance, kAngularTolerance);
  Junction* junction = rg.NewJunction(api::JunctionId{"j:1"});

  EXPECT_EQ(junction->road_geometry(), &rg);
  EXPECT_EQ(junction->id(), api::JunctionId("j:1"));
}

// The following tests Segment creation, getters and index constraints.
GTEST_TEST(RNDFJunctionTest, SegmentTest) {
  RoadGeometry rg(api::RoadGeometryId{"RG-SegmentTest"},
                  kLinearTolerance, kAngularTolerance);
  Junction* junction = rg.NewJunction(api::JunctionId{"j:1"});

  EXPECT_EQ(junction->num_segments(), 0);

  Segment* s1 = junction->NewSegment(api::SegmentId{"s:1"});
  EXPECT_EQ(junction->num_segments(), 1);
  EXPECT_EQ(junction->segment(0), s1);

  Segment* s2 = junction->NewSegment(api::SegmentId{"s:2"});
  EXPECT_EQ(junction->num_segments(), 2);
  EXPECT_EQ(junction->segment(1), s2);

  EXPECT_THROW(junction->segment(-1), std::runtime_error);
  EXPECT_THROW(junction->segment(2), std::runtime_error);
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
