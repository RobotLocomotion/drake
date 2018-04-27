/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/connection.h"
/* clang-format on */

#include <cmath>
#include <ostream>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/test_utilities/multilane_types_compare.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

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
  const double kLaneWidth{2.};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};
  const double kHeading{-M_PI / 4.};
  const EndpointXy kStartXy{20., 30., kHeading};
  const Endpoint kStartEndpoint{kStartXy, kLowFlatZ};
  const double kZeroTolerance{0.};
  const double kVeryExact{1e-12};
};

TEST_F(MultilaneConnectionTest, ArcAccessors) {
  const std::string kId{"arc_connection"};
  const double kRadius{10. * std::sqrt(2.)};
  const double kDTheta{M_PI / 2.};
  const ArcOffset kArcOffset(kRadius, kDTheta);
  const Endpoint kEndEndpoint{{40., 30., kHeading + kDTheta}, kLowFlatZ};

  const Connection dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0,
                       kLaneWidth, kLeftShoulder, kRightShoulder, kArcOffset);
  EXPECT_EQ(dut.type(), Connection::Type::kArc);
  EXPECT_EQ(dut.id(), kId);
  EXPECT_TRUE(
      test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kZeroTolerance));
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.r0(), kR0);
  EXPECT_EQ(dut.lane_width(), kLaneWidth);
  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
  EXPECT_EQ(dut.radius(), kRadius);
  EXPECT_EQ(dut.d_theta(), kDTheta);
  EXPECT_EQ(dut.r_min(), kR0 - kLaneWidth / 2. - kRightShoulder);
  EXPECT_EQ(dut.r_max(),
            kR0 + kLaneWidth * (static_cast<double>(kNumLanes - 1) + .5) +
                kLeftShoulder);
  EXPECT_TRUE(
      test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kVeryExact));
  EXPECT_EQ(dut.lane_offset(0), kR0);
  EXPECT_EQ(dut.lane_offset(1), kR0 + kLaneWidth);
  EXPECT_EQ(dut.lane_offset(2), kR0 + 2. * kLaneWidth);
}

TEST_F(MultilaneConnectionTest, LineAccessors) {
  const std::string kId{"line_connection"};
  const Endpoint kEndEndpoint{{50., 0., kHeading}, kLowFlatZ};

  const double kLineLength{30. * std::sqrt(2.)};
  const Connection dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0,
                       kLaneWidth, kLeftShoulder, kRightShoulder, kLineLength);
  EXPECT_EQ(dut.type(), Connection::Type::kLine);
  EXPECT_EQ(dut.id(), kId);
  EXPECT_TRUE(
      test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kZeroTolerance));
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.r0(), kR0);
  EXPECT_EQ(dut.lane_width(), kLaneWidth);
  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
  EXPECT_EQ(dut.line_length(), kLineLength);
  EXPECT_EQ(dut.r_min(), kR0 - kLaneWidth / 2. - kRightShoulder);
  EXPECT_EQ(dut.r_max(),
            kR0 + kLaneWidth * (static_cast<double>(kNumLanes - 1) + .5) +
                kLeftShoulder);
  EXPECT_TRUE(
      test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kVeryExact));
  EXPECT_EQ(dut.lane_offset(0), kR0);
  EXPECT_EQ(dut.lane_offset(1), kR0 + kLaneWidth);
  EXPECT_EQ(dut.lane_offset(2), kR0 + 2. * kLaneWidth);
}

// Checks RoadCurve creation.
//
// Literals for elevation and superelevation polynomials below have been derived
// in Octave running the following code snippet. Variables with '0' suffix refer
// to start EndpointZ and with '1' suffix refer to end EndpointZ. Replace 'y'
// variables by 'z' related ones and by 'theta' to compute elevation and
// superelevation polynomial coefficients respectively.
//
// % Sets the value of the planar length.
// d_x = 10. * sqrt(2.) * pi / 2.;
// a = y_0 / d_x
// b = y_dot_0 / d_x
// c = 3 * (y_1 - y_0) / d_x - 2 * y_dot_0 - y_dot_1
// d = y_dot_0 + y_dot_1 - 2 * (y_1 - y_0)
TEST_F(MultilaneConnectionTest, ArcRoadCurveValidation) {
  const std::string kId{"arc_connection"};
  const double kRadius{10. * std::sqrt(2.)};
  const double kDTheta{M_PI / 2.};
  const ArcOffset kArcOffset(kRadius, kDTheta);
  const Endpoint kEndEndpoint{{40., 30., kHeading + kDTheta}, kLowFlatZ};

  const Connection flat_dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0,
                            kLaneWidth, kLeftShoulder, kRightShoulder,
                            kArcOffset);
  std::unique_ptr<RoadCurve> road_curve = flat_dut.CreateRoadCurve();
  EXPECT_NE(dynamic_cast<ArcRoadCurve*>(road_curve.get()), nullptr);
  // Checks that the road curve starts and ends at given endpoints.
  const Vector3<double> flat_origin = road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(flat_dut.start().xy().x(), flat_dut.start().xy().y(),
                      flat_dut.start().z().z()),
      flat_origin, kZeroTolerance));
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(),
                      kStartEndpoint.z().z()),
      flat_origin, kZeroTolerance));
  const Vector3<double> flat_end = road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(flat_dut.end().xy().x(), flat_dut.end().xy().y(),
                      flat_dut.end().z().z()),
      flat_end, kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(kEndEndpoint.xy().x(), kEndEndpoint.xy().y(),
                      kEndEndpoint.z().z()),
      flat_end, kVeryExact));
  // Checks that elevation and superelevation polynomials are correctly built
  // for the trivial case of a flat dut.
  EXPECT_EQ(road_curve->elevation().a(), 0.);
  EXPECT_EQ(road_curve->elevation().b(), 0.);
  EXPECT_EQ(road_curve->elevation().c(), 0.);
  EXPECT_EQ(road_curve->elevation().d(), 0.);
  EXPECT_EQ(road_curve->superelevation().a(), 0.);
  EXPECT_EQ(road_curve->superelevation().b(), 0.);
  EXPECT_EQ(road_curve->superelevation().c(), 0.);
  EXPECT_EQ(road_curve->superelevation().d(), 0.);

  // Creates a new complex dut with cubic elevation and superelevation.
  const Endpoint kEndElevatedEndpoint{{40., 30., kHeading + kDTheta},
                                      {5., 1., M_PI / 6., 1.}};
  const Connection complex_dut(kId, kStartEndpoint, kEndElevatedEndpoint.z(),
                               kNumLanes, kR0, kLaneWidth, kLeftShoulder,
                               kRightShoulder, kArcOffset);
  std::unique_ptr<RoadCurve> complex_road_curve = complex_dut.CreateRoadCurve();
  // Checks that the road curve starts and ends at given endpoints.
  const Vector3<double> complex_origin =
      complex_road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(complex_dut.start().xy().x(),
                                              complex_dut.start().xy().y(),
                                              complex_dut.start().z().z()),
                              complex_origin, kZeroTolerance));
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(),
                      kStartEndpoint.z().z()),
      complex_origin, kZeroTolerance));
  const Vector3<double> complex_end = complex_road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(complex_dut.end().xy().x(), complex_dut.end().xy().y(),
                      complex_dut.end().z().z()),
      complex_end, kVeryExact));
  EXPECT_TRUE(CompareMatrices(Vector3<double>(kEndElevatedEndpoint.xy().x(),
                                              kEndElevatedEndpoint.xy().y(),
                                              kEndElevatedEndpoint.z().z()),
                              complex_end, kVeryExact));

  EXPECT_NEAR(complex_road_curve->elevation().a(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->elevation().c(), -0.32476276288217043,
              kVeryExact);
  EXPECT_NEAR(complex_road_curve->elevation().d(), 0.549841841921447,
              kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().a(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().b(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().c(), -0.9292893218813453,
              kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().d(), 0.9528595479208968,
              kVeryExact);
}

TEST_F(MultilaneConnectionTest, LineRoadCurveValidation) {
  const std::string kId{"line_connection"};
  const Endpoint kEndEndpoint{{50., 0., kHeading}, kLowFlatZ};
  const double kLineLength{30. * std::sqrt(2.)};
  const Connection flat_dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0,
                            kLaneWidth, kLeftShoulder, kRightShoulder,
                            kLineLength);
  std::unique_ptr<RoadCurve> road_curve = flat_dut.CreateRoadCurve();
  EXPECT_NE(dynamic_cast<LineRoadCurve*>(road_curve.get()), nullptr);

  // Checks that the road curve starts and ends at given endpoints.
  const Vector3<double> flat_origin = road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(flat_dut.start().xy().x(), flat_dut.start().xy().y(),
                      flat_dut.start().z().z()),
      flat_origin, kZeroTolerance));
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(),
                      kStartEndpoint.z().z()),
      flat_origin, kZeroTolerance));
  const Vector3<double> flat_end = road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(flat_dut.end().xy().x(), flat_dut.end().xy().y(),
                      flat_dut.end().z().z()),
      flat_end, kVeryExact));
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(kEndEndpoint.xy().x(), kEndEndpoint.xy().y(),
                      kEndEndpoint.z().z()),
      flat_end, kVeryExact));
  // Checks that elevation and superelevation polynomials are correctly built
  // for the trivial case of a flat dut.
  EXPECT_EQ(road_curve->elevation().a(), 0.);
  EXPECT_EQ(road_curve->elevation().b(), 0.);
  EXPECT_EQ(road_curve->elevation().c(), 0.);
  EXPECT_EQ(road_curve->elevation().d(), 0.);
  EXPECT_EQ(road_curve->superelevation().a(), 0.);
  EXPECT_EQ(road_curve->superelevation().b(), 0.);
  EXPECT_EQ(road_curve->superelevation().c(), 0.);
  EXPECT_EQ(road_curve->superelevation().d(), 0.);

  // Creates a new complex dut with cubic elevation and superelevation.
  const Endpoint kEndElevatedEndpoint{{50., 0., kHeading},
                                      {5., 1., M_PI / 6., 1.}};
  const Connection complex_dut(kId, kStartEndpoint, kEndElevatedEndpoint.z(),
                               kNumLanes, kR0, kLaneWidth, kLeftShoulder,
                               kRightShoulder, kLineLength);
  std::unique_ptr<RoadCurve> complex_road_curve = complex_dut.CreateRoadCurve();

  // Checks that the road curve starts and ends at given endpoints.
  const Vector3<double> complex_origin =
      complex_road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(CompareMatrices(Vector3<double>(complex_dut.start().xy().x(),
                                              complex_dut.start().xy().y(),
                                              complex_dut.start().z().z()),
                              complex_origin, kZeroTolerance));
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(),
                      kStartEndpoint.z().z()),
      complex_origin, kZeroTolerance));
  const Vector3<double> complex_end = complex_road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(CompareMatrices(
      Vector3<double>(complex_dut.end().xy().x(), complex_dut.end().xy().y(),
                      complex_dut.end().z().z()),
      complex_end, kVeryExact));
  EXPECT_TRUE(CompareMatrices(Vector3<double>(kEndElevatedEndpoint.xy().x(),
                                              kEndElevatedEndpoint.xy().y(),
                                              kEndElevatedEndpoint.z().z()),
                              complex_end, kVeryExact));

  EXPECT_NEAR(complex_road_curve->elevation().a(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->elevation().b(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->elevation().c(), -0.646446609406726,
              kVeryExact);
  EXPECT_NEAR(complex_road_curve->elevation().d(), 0.764297739604484,
              kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().a(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().b(), 0., kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().c(), -0.962975975515347,
              kVeryExact);
  EXPECT_NEAR(complex_road_curve->superelevation().d(), 0.975317317010231,
              kVeryExact);
}

// Lane Endpoints with different EndpointZ. Those are selected to cover
// different combinations of elevation and superelevation polynomials.

// Groups test parameters.
struct EndpointZTestParameters{
  EndpointZTestParameters() = default;
  EndpointZTestParameters(const EndpointZ& _start_z, const EndpointZ& _end_z,
                          double _r0, int _num_lanes)
      : start_z(_start_z), end_z(_end_z), r0(_r0), num_lanes(_num_lanes) {}

  EndpointZ start_z{};
  EndpointZ end_z{};
  double r0{};
  int num_lanes{};
};

// Stream insertion operator overload for EndpointZTestParameters
// instances. Necessary for gtest printouts that would otherwise fail
// at properly printing the struct's bytes (its default behavior when
// no stream insertion operator overload is present) and trigger Valgrind
// errors.
std::ostream& operator<<(std::ostream& stream,
                         const EndpointZTestParameters& endpoint_z_test_param) {
  return stream << "EndpointZTestParameters( start_z: ("
                << endpoint_z_test_param.start_z  << "), end_z: ("
                << endpoint_z_test_param.end_z << "), r0: "
                << endpoint_z_test_param.r0 << ", num_lanes: "
                << endpoint_z_test_param.num_lanes << ")";
}

// Groups common test constants as well as each test case parameters.
class MultilaneConnectionEndpointZTest
    : public ::testing::TestWithParam<EndpointZTestParameters> {
 protected:
  void SetUp() override {
    const EndpointZTestParameters parameters = this->GetParam();
    start_z = parameters.start_z;
    end_z = parameters.end_z;
    r0 = parameters.r0;
    num_lanes = parameters.num_lanes;
    start_endpoint = {kStartXy, start_z};
  }

  const double kLeftShoulder{1.};
  const double kRightShoulder{1.5};
  const double kLaneWidth{2.};
  const double kHeading{-M_PI / 4.};
  const EndpointXy kStartXy{20., 30., kHeading};
  EndpointZ start_z{};
  EndpointZ end_z{};
  double r0{};
  int num_lanes{};
  Endpoint start_endpoint{};
  const double kVeryExact{1e-12};
};

TEST_P(MultilaneConnectionEndpointZTest, ArcLaneEndpoints) {
  const std::string kId{"arc_connection"};
  const double kCenterX{30.};
  const double kCenterY{40.};
  const double kRadius{10. * std::sqrt(2.)};
  const double kDTheta{M_PI / 2.};
  const ArcOffset kArcOffset(kRadius, kDTheta);
  const Connection dut(kId, start_endpoint, end_z, num_lanes, r0, kLaneWidth,
                       kLeftShoulder, kRightShoulder, kArcOffset);
  const double kTheta0{kHeading - M_PI / 2.};

  // Wraps angles in [-π, π) range.
  auto wrap = [](double theta) {
    double theta_new = std::fmod(theta + M_PI, 2. * M_PI);
    if (theta_new < 0.) theta_new += 2. * M_PI;
    return theta_new - M_PI;
  };

  for (int i = 0; i < num_lanes; i++) {
    // Start endpoints.
    const double start_radius =
        kRadius - (r0 + static_cast<double>(i) * kLaneWidth) *
                  std::cos(start_z.theta());
    const Endpoint lane_start{
        {kCenterX + start_radius * std::cos(kTheta0),
         kCenterY + start_radius * std::sin(kTheta0),
         wrap(kTheta0 + M_PI / 2.)},
        {start_z.z(), start_z.z_dot() * kRadius / start_radius,
         start_z.theta(), start_z.theta_dot() * kRadius / start_radius}};
    EXPECT_TRUE(
        test::IsEndpointClose(dut.LaneStart(i), lane_start, kVeryExact));
    // End endpoints.
    const double end_radius =
        kRadius - (r0 + static_cast<double>(i) * kLaneWidth) *
                  std::cos(end_z.theta());
    const Endpoint lane_end{
        {kCenterX + end_radius * std::cos(kTheta0 + kDTheta),
         kCenterY + end_radius * std::sin(kTheta0 + kDTheta),
         wrap(kTheta0 + kDTheta + M_PI / 2.)},
        {end_z.z(), end_z.z_dot() * kRadius / end_radius,
         end_z.theta(), end_z.theta_dot() * kRadius / end_radius}};
    EXPECT_TRUE(test::IsEndpointClose(dut.LaneEnd(i), lane_end, kVeryExact));
  }
}

TEST_P(MultilaneConnectionEndpointZTest, LineLaneEndpoints) {
  const std::string kId{"line_connection"};
  const double kLineLength{25. * std::sqrt(2.)};
  const Connection dut(kId, start_endpoint, end_z, num_lanes, r0, kLaneWidth,
                       kLeftShoulder, kRightShoulder, kLineLength);
  const Vector2<double> kDirection{45. - kStartXy.x(), 5. - kStartXy.y()};
  const Vector2<double> kNormalDirection =
      Vector2<double>(kDirection.y(), -kDirection.x()).normalized();

  for (int i = 0; i < num_lanes; i++) {
    // Start endpoints.
    const double offset = r0 + static_cast<double>(i) * kLaneWidth;
    const Endpoint lane_start{
        {kStartXy.x() - offset * kNormalDirection.x(),
         kStartXy.y() - offset * kNormalDirection.y(), kHeading}, start_z};
    EXPECT_TRUE(
        test::IsEndpointClose(dut.LaneStart(i), lane_start, kVeryExact));
    // End endpoints.
    const Endpoint lane_end{
        {kStartXy.x() - offset * kNormalDirection.x() + kDirection.x(),
         kStartXy.y() - offset * kNormalDirection.y() + kDirection.y(),
         kHeading}, end_z};
    EXPECT_TRUE(test::IsEndpointClose(dut.LaneEnd(i), lane_end, kVeryExact));
  }
}

INSTANTIATE_TEST_CASE_P(EndpointZ, MultilaneConnectionEndpointZTest,
    testing::Values(EndpointZTestParameters(EndpointZ(0., 0., 0., 0.),
                                            EndpointZ(0., 0., 0., 0.),
                                            0., 1),
                    EndpointZTestParameters(EndpointZ(0., 0., 0., 0.),
                                            EndpointZ(0., 0., 0., 0.),
                                            2., 3),
                    EndpointZTestParameters(EndpointZ(1., 0., 0., 0.),
                                            EndpointZ(1., 0., 0., 0.),
                                            2., 3),
                    EndpointZTestParameters(EndpointZ(1., 1., 0., 0.),
                                            EndpointZ(5., 1., 0., 0.),
                                            2., 3),
                    EndpointZTestParameters(EndpointZ(0., 0., M_PI / 6., 0.),
                                            EndpointZ(0., 0., M_PI / 6., 0.),
                                            0., 1),
                    EndpointZTestParameters(EndpointZ(1., 1., M_PI / 6., 0.),
                                            EndpointZ(5., 1., M_PI / 6., 0.),
                                            0., 1),
                    EndpointZTestParameters(EndpointZ(1., 1., M_PI / 3., 1.),
                                            EndpointZ(5., 1., M_PI / 6., 1.),
                                            0., 1)));

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
