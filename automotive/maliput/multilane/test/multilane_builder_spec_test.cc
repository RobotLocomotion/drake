/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/builder_spec.h"
/* clang-format on */

#include <string>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

using Which = api::LaneEnd::Which;

// StartReferenceSpec using an Endpoint.
GTEST_TEST(StartReferenceSpecTest, Endpoint) {
  const Endpoint point{{1., 2., 3.}, {4., 5., 6., 7.}};
  const double kVeryExact{1e-15};

  const StartReference::Spec forward_dut =
      StartReference().at(point, Direction::kForward);
  EXPECT_TRUE(test::IsEndpointClose(forward_dut.endpoint(), point, kVeryExact));

  const StartReference::Spec reversed_dut =
      StartReference().at(point, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointClose(reversed_dut.endpoint(), point.reverse(),
                                    kVeryExact));
}

// StartReferenceSpec using a connection's reference curve.
GTEST_TEST(StartReferenceSpecTest, Connection) {
  const EndpointZ kFlatEndpointZ{0., 0., 0., 0.};
  const Endpoint kStartEndpoint{{1., 2., 3.}, kFlatEndpointZ};
  const Connection conn("conn", kStartEndpoint, kFlatEndpointZ, 2, 0., 1., 1.5,
                        1.5, LineOffset(10.));
  const double kVeryExact{1e-15};

  const StartReference::Spec forward_start_dut =
      StartReference().at(conn, Which::kStart, Direction::kForward);
  EXPECT_TRUE(test::IsEndpointClose(forward_start_dut.endpoint(), conn.start(),
                                    kVeryExact));

  const StartReference::Spec reversed_start_dut =
      StartReference().at(conn, Which::kStart, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointClose(reversed_start_dut.endpoint(),
                                    conn.start().reverse(), kVeryExact));

  const StartReference::Spec forward_end_dut =
      StartReference().at(conn, Which::kFinish, Direction::kForward);
  EXPECT_TRUE(test::IsEndpointClose(forward_end_dut.endpoint(), conn.end(),
                                    kVeryExact));

  const StartReference::Spec reversed_end_dut =
      StartReference().at(conn, Which::kFinish, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointClose(reversed_end_dut.endpoint(),
                                    conn.end().reverse(), kVeryExact));
}

// EndReferenceSpec using an EndpointZ.
GTEST_TEST(EndReferenceSpecTest, Endpoint) {
  const EndpointZ z_point{4., 5., 6., 7.};
  const double kVeryExact{1e-15};

  const EndReference::Spec z_forward_dut =
      EndReference().z_at(z_point, Direction::kForward);
  EXPECT_TRUE(
      test::IsEndpointZClose(z_forward_dut.endpoint_z(), z_point, kVeryExact));

  const EndReference::Spec z_reversed_dut =
      EndReference().z_at(z_point, Direction::kReverse);
  EXPECT_TRUE(test::IsEndpointZClose(z_reversed_dut.endpoint_z(),
                                     z_point.reverse(), kVeryExact));
}

// LaneLayout check.
GTEST_TEST(LaneLayeoutTest, ConstructorAndAccessors) {
  const double kLeftShoulder{6.789};
  const double kRightShoulder{0.123};
  const int kNumLanes{4};
  const int kRefLane{2};
  const double kRefR0{-3.14};
  const LaneLayout dut(kLeftShoulder, kRightShoulder, kNumLanes, kRefLane,
                       kRefR0);

  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.ref_lane(), kRefLane);
  EXPECT_EQ(dut.ref_r0(), kRefR0);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
