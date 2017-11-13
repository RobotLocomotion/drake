/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/connection.h"
/* clang-format on */

#include <cmath>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"

namespace drake {
namespace maliput {
namespace multilane {

// EndpointXy checks.
GTEST_TEST(EndpointXyTest, DefaultConstructor) {
  const EndpointXy dut{};
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.heading(), 0.);
}

GTEST_TEST(EndpointXyTest, ParametrizedConstructor) {
  const EndpointXy dut{1., 2., M_PI / 4.};
  EXPECT_EQ(dut.x(), 1.);
  EXPECT_EQ(dut.y(), 2.);
  EXPECT_EQ(dut.heading(), M_PI / 4.);
}

GTEST_TEST(EndpointXyTest, Reverse) {
  const EndpointXy dut{1., 2., M_PI / 7.123456};
  const double kVeryExact{1e-15};
  EXPECT_TRUE(test::IsEndpointXyClose(dut.reverse(),
                                      {1., 2., -M_PI * (1. - 1. / 7.123456)},
                                      kVeryExact));
}

// EndpointZ checks.
GTEST_TEST(EndpointZTest, DefaultConstructor) {
  const EndpointZ dut{};
  EXPECT_EQ(dut.z(), 0.);
  EXPECT_EQ(dut.z_dot(), 0.);
  EXPECT_EQ(dut.theta(), 0.);
  EXPECT_EQ(dut.theta_dot(), 0.);
}

GTEST_TEST(EndpointZTest, ParametrizedConstructor) {
  const EndpointZ dut{1., 2., M_PI / 4., M_PI / 2.};
  EXPECT_EQ(dut.z(), 1.);
  EXPECT_EQ(dut.z_dot(), 2.);
  EXPECT_EQ(dut.theta(), M_PI / 4.);
  EXPECT_EQ(dut.theta_dot(), M_PI / 2.);
}

GTEST_TEST(EndpointZTest, Reverse) {
  const EndpointZ dut{1., 2., M_PI / 4., M_PI / 2.};
  const double kZeroTolerance{0.};
  EXPECT_TRUE(test::IsEndpointZClose(
      dut.reverse(), {1., -2., -M_PI / 4., M_PI / 2.}, kZeroTolerance));
}

// ArcOffset checks.
GTEST_TEST(ArcOffsetTest, DefaultConstructor) {
  const ArcOffset dut{};
  EXPECT_EQ(dut.radius(), 0.);
  EXPECT_EQ(dut.d_theta(), 0.);
}

GTEST_TEST(ArcOffsetTest, ParametrizedConstructor) {
  const ArcOffset dut{1., M_PI / 4.};
  EXPECT_EQ(dut.radius(), 1.);
  EXPECT_EQ(dut.d_theta(), M_PI / 4.);
}

// Connection checks.
class MultilaneConnectionTest : public ::testing::Test {
 protected:
  const double kR0{2.};
  const int kNumLanes{3};
  const double kLeftShoulder{1.};
  const double kRightShoulder{1.5};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};
  const double kHeading{-M_PI / 4.};
  const EndpointXy kStartXy{20., 30., kHeading};
  const Endpoint kStartEndpoint{kStartXy, kLowFlatZ};
  const double kZeroTolerance{0.};
};

TEST_F(MultilaneConnectionTest, ArcAccessors) {
  const std::string kId{"arc_connection"};
  const double kCenterX{30.};
  const double kCenterY{40.};
  const double kRadius{10. * std::sqrt(2.)};
  const double kDTheta{M_PI / 2.};
  const Endpoint kEndEndpoint{{40., 30., kHeading + kDTheta}, kLowFlatZ};

  const Connection dut(kId, kStartEndpoint, kEndEndpoint, kNumLanes, kR0,
                       kLeftShoulder, kRightShoulder, kCenterX, kCenterY,
                       kRadius, kDTheta);
  EXPECT_EQ(dut.type(), Connection::Type::kArc);
  EXPECT_EQ(dut.id(), kId);
  EXPECT_TRUE(
      test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kZeroTolerance));
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.r0(), kR0);
  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
  EXPECT_EQ(dut.cx(), kCenterX);
  EXPECT_EQ(dut.cy(), kCenterY);
  EXPECT_EQ(dut.radius(), kRadius);
  EXPECT_EQ(dut.d_theta(), kDTheta);
}

TEST_F(MultilaneConnectionTest, LineAccessors) {
  const std::string kId{"line_connection"};
  const Endpoint kEndEndpoint{{50., 0., kHeading}, kLowFlatZ};
  const Connection dut(kId, kStartEndpoint, kEndEndpoint, kNumLanes, kR0,
                       kLeftShoulder, kRightShoulder);
  EXPECT_EQ(dut.type(), Connection::Type::kLine);
  EXPECT_EQ(dut.id(), kId);
  EXPECT_TRUE(
      test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kZeroTolerance));
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.r0(), kR0);
  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
