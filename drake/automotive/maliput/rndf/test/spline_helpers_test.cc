#include "drake/automotive/maliput/rndf/spline_helpers.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

namespace drake {
namespace maliput {
namespace rndf {

// We use this tolerance to match the interpolated positions given by the path
// length interpolator wrappers.
const double kLinearTolerance = 1e-4;
// We use this step to sample the spline at this rate, to get the closest path
// length coordinate to a point in space.
const double kLinearStep = 1e-2;

#define EXPECT_IGN_VECTOR_NEAR(actual_arg, expected_arg, tolerance_arg) \
  do {                                                                  \
    const ignition::math::Vector3d actual(actual_arg);                  \
    const ignition::math::Vector3d expected(expected_arg);              \
    const double tolerance(tolerance_arg);                              \
    EXPECT_NEAR(actual.X(), expected.X(), tolerance);                   \
    EXPECT_NEAR(actual.Y(), expected.Y(), tolerance);                   \
    EXPECT_NEAR(actual.Z(), expected.Z(), tolerance);                   \
  } while (0)

// This is a wrapper to easily create an ignition::math::Spline.
// @param points a tuple consisting of two ignition::math::Vector3d. The
// first one is the position of the point and the second one is the tangent
// vector or the spline first derivative at that point.
// @return a pointer to an ignition::math::Spline object.
std::unique_ptr<ignition::math::Spline> CreateSpline(
    const std::vector<std::tuple<ignition::math::Vector3d,
                                 ignition::math::Vector3d>>& points) {
  auto spline = std::make_unique<ignition::math::Spline>();
  spline->Tension(1.0);
  spline->AutoCalculate(true);
  for (const auto& point : points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline;
}

// Tests the InverseFunctionInterpolator exceptions.
GTEST_TEST(RNDFSplineHelperTest, ExceptionsInInverseFunctionInterpolator) {
  // Error boundary set to negative.
  EXPECT_THROW(
      InverseFunctionInterpolator([](double t) { return t; }, 0., 1., -1.),
      std::runtime_error);
  // xmin is equal to xmax
  EXPECT_THROW(
      InverseFunctionInterpolator([](double t) { return t; }, 0., 0., -1.),
      std::runtime_error);
  // xmin is greater than xmax
  EXPECT_THROW(
      InverseFunctionInterpolator([](double t) { return t; }, 1., 0., -1.),
      std::runtime_error);

  InverseFunctionInterpolator function_interpolator([](double t) { return t; },
                                                    0., 1., kLinearTolerance);
  // Derivative number set to negative.
  EXPECT_THROW(function_interpolator.InterpolateMthDerivative(-1, 0.),
               std::runtime_error);
  // Minimum bound out of range.
  EXPECT_THROW(function_interpolator.InterpolateMthDerivative(0, -1.),
               std::runtime_error);
  // Maximum bound out of range.
  EXPECT_THROW(function_interpolator.InterpolateMthDerivative(0, 2.),
               std::runtime_error);
}

// Tests the ArcLengthParameterizedSpline exceptions.
GTEST_TEST(RNDFSplineHelperTest, ExceptionsInArcLengthParameterizedSpline) {
  std::unique_ptr<ignition::math::Spline> spline;
  // Spline pointer set to nullptr
  EXPECT_THROW(
      ArcLengthParameterizedSpline(std::move(spline), kLinearTolerance),
      std::runtime_error);
}

// Checks the spline interpolator class and compares its resolution against
// a straight line lane.
GTEST_TEST(RNDFSplineHelperTest, StraightLine) {
  const ignition::math::Vector3d kTangent(20., 0., 0.);
  const ignition::math::Vector3d kFirstDerivativeInterpolation(1, 0., 0.);
  const ignition::math::Vector3d kSecondDerivativeInterpolation(0., 0., 0.);
  const ignition::math::Vector3d kStartPoint(0., 0.0, 0.0);
  const ignition::math::Vector3d kEndPoint(20., 0.0, 0.0);

  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(std::make_tuple(kStartPoint, kTangent));
  control_points.push_back(std::make_tuple(kEndPoint, kTangent));

  std::unique_ptr<ignition::math::Spline> spline = CreateSpline(control_points);
  auto arc_length_param_spline = std::make_unique<ArcLengthParameterizedSpline>(
      std::move(spline), kLinearTolerance);

  const double length = arc_length_param_spline->BaseSpline()->ArcLength();
  ignition::math::Vector3d p(kStartPoint);
  for (double l = 0.0; l < length; l += (length / 10.)) {
    p.X() = l;
    EXPECT_IGN_VECTOR_NEAR(
        arc_length_param_spline->InterpolateMthDerivative(0, l), p,
        kLinearTolerance);
    EXPECT_IGN_VECTOR_NEAR(
        arc_length_param_spline->InterpolateMthDerivative(1, l),
        kFirstDerivativeInterpolation, kLinearTolerance);
    EXPECT_IGN_VECTOR_NEAR(
        arc_length_param_spline->InterpolateMthDerivative(2, l),
        kSecondDerivativeInterpolation, kLinearTolerance);
  }
  EXPECT_IGN_VECTOR_NEAR(
      arc_length_param_spline->InterpolateMthDerivative(0, length), kEndPoint,
      kLinearTolerance);
  EXPECT_IGN_VECTOR_NEAR(
      arc_length_param_spline->InterpolateMthDerivative(1, length),
      kFirstDerivativeInterpolation, kLinearTolerance);
  EXPECT_IGN_VECTOR_NEAR(
      arc_length_param_spline->InterpolateMthDerivative(2, length),
      kSecondDerivativeInterpolation, kLinearTolerance);
}

// Tests a set of points and the result of the path length distance that returns
// a Spline interpolated point closest to it.
GTEST_TEST(RNDFSplineHelperTest, StraightSplineFindClosesPointTo) {
  std::vector<std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(0.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
      std::make_tuple(ignition::math::Vector3d(20.0, 0.0, 0.0),
                      ignition::math::Vector3d(10.0, 0.0, 0.0)));

  std::unique_ptr<ignition::math::Spline> spline = CreateSpline(control_points);
  auto arc_length_param_spline = std::make_unique<ArcLengthParameterizedSpline>(
      std::move(spline), kLinearTolerance);
  // Border checks
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(0.0, 5.0, 0.0), kLinearStep),
              0., kLinearTolerance);
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(0.0, -5.0, 0.0), kLinearStep),
              0., kLinearTolerance);
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(20.0, 5.0, 0.0), kLinearStep),
              20., kLinearTolerance);
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(20.0, -5.0, 0.0), kLinearStep),
              20., kLinearTolerance);
  // Middle checks
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(10.0, 5.0, 0.0), kLinearStep),
              10., kLinearTolerance);
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(10.0, -5.0, 0.0), kLinearStep),
              10., kLinearTolerance);
  // Before and after checks
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(-5, -5.0, 0.0), kLinearStep),
              0., kLinearTolerance);
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(25, 5.0, 0.0), kLinearStep),
              20., kLinearTolerance);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
