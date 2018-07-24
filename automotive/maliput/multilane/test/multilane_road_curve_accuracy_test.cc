/* clang-format off to disable clang-format-includes */
#include "drake/automotive/maliput/multilane/road_curve.h"
/* clang-format on */

#include <memory>
#include <ostream>
#include <tuple>
#include <utility>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/cubic_polynomial.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/automotive/maliput/multilane/test_utilities/multilane_brute_force_integral.h"

namespace drake {
namespace maliput {
namespace multilane {
namespace {

// Checks brute force integral computations against known
// path length arc road curves.
GTEST_TEST(BruteForceIntegralTest, ArcRoadCurvePathLength) {
  const double kAccuracy{1e-12};

  const double kRadius{10.0};
  const double kTheta0{M_PI / 4.0};
  const double kTheta1{3.0 * M_PI / 4.0};
  const double kDTheta{kTheta1 - kTheta0};
  const Vector2<double> kCenter{10.0, 10.0};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{
    ComputationPolicy::kPreferAccuracy};
  const CubicPolynomial zp(0., 0., 0., 0.);
  const double kP0{0.};
  const double kP1{1.};
  const double kR{0.};
  const double kH{0.};

  const ArcRoadCurve rc(kCenter, kRadius, kTheta0, kDTheta, zp, zp,
                        kLinearTolerance, kScaleLength, kComputationPolicy);

  // A k = 0 order approximation uses a single segment, as n = 2^k,
  // where n is the segment count.
  double maximum_step = 0;
  const double path_length_zero_order_approx =
      test::BruteForcePathLengthIntegral(
          rc, kP0, kP1, kR, kH, 0, &maximum_step);
  const double path_length_zero_order = std::sin(M_PI / 4.) * kRadius * 2.;
  EXPECT_NEAR(path_length_zero_order_approx, path_length_zero_order, kAccuracy);
  EXPECT_NEAR(maximum_step, path_length_zero_order, kAccuracy);

  int k_order_hint = 0;
  const double tolerance = .01 * rc.CalcSFromP(1., 0.);
  const double path_length_adaptive_approx =
      test::AdaptiveBruteForcePathLengthIntegral(
          rc, kP0, kP1, kR, kH, tolerance, &k_order_hint);
  EXPECT_NEAR(path_length_adaptive_approx, kRadius * M_PI / 2., tolerance);
}

// A test fixture for RoadCurve computation accuracy tests.
class RoadCurveAccuracyTest
    : public ::testing::TestWithParam<std::shared_ptr<RoadCurve>> {};

// Checks that path length computations are within tolerance.
TEST_P(RoadCurveAccuracyTest, PathLengthAccuracy) {
  const double kMinimumP = 0.;
  const double kMaximumP = 1.;
  const double kPStep = 0.2;

  const double kMinimumR = -5.0;
  const double kMaximumR = 5.0;
  const double kRStep = 2.5;

  const double kH = 0.0;

  std::shared_ptr<RoadCurve> road_curve = GetParam();
  const double kTolerance = road_curve->linear_tolerance()
                            / road_curve->scale_length();
  for (double r = kMinimumR; r <= kMaximumR; r += kRStep) {
    int k_order = 0;
    for (double p = kMinimumP; p <= kMaximumP; p += kPStep) {
      const double k_order_s_approximation =
          test::AdaptiveBruteForcePathLengthIntegral(
              *road_curve, kMinimumP, p, r, kH,
              road_curve->linear_tolerance(), &k_order);
      const double relative_error =
          (k_order_s_approximation != 0.0) ?
          (road_curve->CalcSFromP(p, r) - k_order_s_approximation) /
          k_order_s_approximation : road_curve->CalcSFromP(p, r);
      EXPECT_LE(relative_error, kTolerance) << fmt::format(
          "Path length estimation with a tolerance of {} "
          "m failed at p = {}, r = {}m, h = {}m with "
          "{} for elevation and {} for superelevation",
          road_curve->linear_tolerance(), p, r, kH, road_curve->elevation(),
          road_curve->superelevation());
    }
  }
}

// Returns an exhaustive combination of CubicPolynomial instances for testing.
std::vector<CubicPolynomial> GetCubicPolynomials() {
  return {
    {0.0, 0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0},
    {1.0, 0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 0.0},
    {1.0, 1.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0},
    {1.0, 0.0, 0.0, 1.0},
    {0.0, 1.0, 0.0, 1.0},
    {1.0, 1.0, 0.0, 1.0},
    {0.0, 0.0, 1.0, 1.0},
    {1.0, 0.0, 1.0, 1.0},
    {0.0, 1.0, 1.0, 1.0},
    {1.0, 1.0, 1.0, 1.0}};
}

// Returns a collection of ArcRoadCurve instances for testing
// that are simple enough for fast analytical computations to be
// accurate.
std::vector<std::shared_ptr<RoadCurve>> GetSimpleLineRoadCurves() {
  const Vector2<double> kStart{1., 1.};
  const Vector2<double> kEnd{10., -8.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const CubicPolynomial zp{0., 0., 0., 0.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    if (elevation_polynomial.order() <= 1) {
      road_curves.push_back(std::make_shared<LineRoadCurve>(
          kStart, kEnd - kStart, elevation_polynomial,
          zp, kLinearTolerance, kScaleLength,
          ComputationPolicy::kPreferSpeed));
    }
  }
  return road_curves;
}


// Returns a collection of LineRoadCurve instances for testing.
std::vector<std::shared_ptr<RoadCurve>> GetLineRoadCurves() {
  const Vector2<double> kStart{0., 0.};
  const Vector2<double> kEnd{10., -8.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    for (const auto& superelevation_polynomial : GetCubicPolynomials()) {
      road_curves.push_back(std::make_shared<LineRoadCurve>(
          kStart, kEnd - kStart, elevation_polynomial,
          superelevation_polynomial, kLinearTolerance, kScaleLength,
          ComputationPolicy::kPreferAccuracy));
    }
  }
  return road_curves;
}


// Returns a collection of ArcRoadCurve instances for testing
// that are simple enough for fast analytical computations to be
// accurate.
std::vector<std::shared_ptr<RoadCurve>> GetSimpleArcRoadCurves() {
  const Vector2<double> kCenter{1., 1.};
  const double kRadius{12.0};
  const double kTheta0{M_PI / 9.};
  const double kDTheta{M_PI / 3.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const CubicPolynomial zp{0., 0., 0., 0.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    if (elevation_polynomial.order() <= 1) {
      road_curves.push_back(std::make_shared<ArcRoadCurve>(
          kCenter, kRadius, kTheta0, kDTheta,
          elevation_polynomial, zp,
          kLinearTolerance, kScaleLength,
          ComputationPolicy::kPreferSpeed));
    }
  }
  return road_curves;
}

// Returns a collection of ArcRoadCurve instances for testing.
std::vector<std::shared_ptr<RoadCurve>> GetArcRoadCurves() {
  const Vector2<double> kCenter{0., 0.};
  const double kRadius{10.0};
  const double kTheta0{M_PI / 6.};
  const double kDTheta{M_PI / 2.};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};

  std::vector<std::shared_ptr<RoadCurve>> road_curves;
  for (const auto& elevation_polynomial : GetCubicPolynomials()) {
    for (const auto& superelevation_polynomial : GetCubicPolynomials()) {
      road_curves.push_back(std::make_shared<ArcRoadCurve>(
        kCenter, kRadius, kTheta0, kDTheta,
        elevation_polynomial, superelevation_polynomial,
        kLinearTolerance, kScaleLength,
        ComputationPolicy::kPreferAccuracy));
    }
  }
  return road_curves;
}


INSTANTIATE_TEST_CASE_P(SimpleAndFastLineRoadCurveAccuracyTest,
                        RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetSimpleLineRoadCurves()));

INSTANTIATE_TEST_CASE_P(ExhaustiveLineRoadCurveAccuracyTest,
                        RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetLineRoadCurves()));

INSTANTIATE_TEST_CASE_P(SimpleAndFastArcRoadCurveAccuracyTest,
                        RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetSimpleArcRoadCurves()));

INSTANTIATE_TEST_CASE_P(ExhaustiveArcRoadCurveAccuracyTest,
                        RoadCurveAccuracyTest,
                        ::testing::ValuesIn(GetArcRoadCurves()));

}  // namespace
}  // namespace multilane
}  // namespace maliput
}  // namespace drake
