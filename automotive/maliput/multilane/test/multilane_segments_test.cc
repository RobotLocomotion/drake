/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/segment.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/test_utilities/maliput_types_compare.h"
#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/junction.h"
#include "drake/automotive/maliput/multilane/lane.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/automotive/maliput/multilane/road_geometry.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

const double kLinearTolerance = 1e-6;
const double kAngularTolerance = 1e-6;
const double kZeroTolerance = 0.;

GTEST_TEST(MultilaneSegmentsTest, MultipleLanes) {
  CubicPolynomial zp{0., 0., 0., 0.};
  const double kR0 = 10.;
  const int kNumLanes = 3;
  const double kRSpacing = 15.;
  const double kRMin = 2.;
  const double kRMax = 42.;
  const double kHalfLaneWidth = 0.5 * kRSpacing;
  const double kMaxHeight = 5.;
  const double kScaleLength = 1.;
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};

  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance,
                  kAngularTolerance);
  std::unique_ptr<RoadCurve> road_curve_1 = std::make_unique<LineRoadCurve>(
      Vector2<double>(100., -75.), Vector2<double>(100., 50.), zp, zp,
      kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s1 =
      rg.NewJunction(api::JunctionId{"j1"})
          ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve_1), kRMin,
                       kRMax, {0., kMaxHeight});
  EXPECT_EQ(s1->id(), api::SegmentId("s1"));
  EXPECT_EQ(s1->num_lanes(), 0);

  Lane* l0 = s1->NewLane(api::LaneId{"l0"}, kR0, {-8., kHalfLaneWidth});
  EXPECT_EQ(s1->num_lanes(), 1);
  EXPECT_EQ(s1->lane(0), l0);
  Lane* l1 = s1->NewLane(api::LaneId{"l1"}, kR0 + kRSpacing,
                         {-kHalfLaneWidth, kHalfLaneWidth});
  EXPECT_EQ(s1->num_lanes(), 2);
  EXPECT_EQ(s1->lane(1), l1);
  Lane* l2 = s1->NewLane(api::LaneId{"l2"}, kR0 + 2. * kRSpacing,
                         {-kHalfLaneWidth, 2.});
  EXPECT_EQ(s1->num_lanes(), kNumLanes);
  EXPECT_EQ(s1->lane(2), l2);

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_TRUE(api::test::IsRBoundsClose(l0->lane_bounds(0.),
                                        {-8., kHalfLaneWidth}, kZeroTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(l0->driveable_bounds(0.), {-8., 32.},
                                        kZeroTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(
      l1->lane_bounds(0.), {-kHalfLaneWidth, kHalfLaneWidth}, kZeroTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->driveable_bounds(0.), {-23., 17.},
                                        kZeroTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(l2->lane_bounds(0.),
                                        {-kHalfLaneWidth, 2.}, kZeroTolerance));
  EXPECT_TRUE(api::test::IsRBoundsClose(l2->driveable_bounds(0.), {-38., 2.},
                                        kZeroTolerance));
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
