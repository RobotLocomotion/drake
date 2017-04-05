#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>


#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/automotive/maliput/rndf/spline_helpers.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-4;
const double kLinearStep = 1e-2;

#define EXPECT_IGN_VECTOR_NEAR(actual, expected, tolerance)  \
  do {                                                      \
    const ignition::math::Vector3d _actual(actual);         \
    const ignition::math::Vector3d _expected(expected);     \
    const double _tolerance(tolerance);                     \
    EXPECT_NEAR(_actual.X(), _expected.X(), _tolerance);    \
    EXPECT_NEAR(_actual.Y(), _expected.Y(), _tolerance);    \
    EXPECT_NEAR(_actual.Z(), _expected.Z(), _tolerance);    \
  } while (0)

std::unique_ptr<ignition::math::Spline>
CreateSpline(
  const std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>> &points) {
  std::unique_ptr<ignition::math::Spline> spline =
    std::make_unique<ignition::math::Spline>();
  spline->Tension(1.0);
  spline->AutoCalculate(true);
  for (const auto &point : points) {
    spline->AddPoint(std::get<0>(point), std::get<1>(point));
  }
  return spline;
}

// It checks the spline interpolator class and compares its resolution against
// a straight line lane.
GTEST_TEST(RNDFSplineHelperTest, StraightLine) {
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));

  std::unique_ptr<ignition::math::Spline> spline =
    CreateSpline(control_points);
  std::unique_ptr<ArcLengthParameterizedSpline> arc_lenght_param_spline =
    std::make_unique<ArcLengthParameterizedSpline>(
      std::move(spline), kLinearTolerance);

  const double length = arc_lenght_param_spline->BaseSpline()->ArcLength();
  ignition::math::Vector3d p(0.0, 0.0, 0.0);
  for (double l = 0.0; l < length; l += 1.0) {
    p.X() = l;
    EXPECT_IGN_VECTOR_NEAR(
      arc_lenght_param_spline->InterpolateMthDerivative(0, l),
      p,
      kLinearTolerance);
  }
  EXPECT_IGN_VECTOR_NEAR(
  arc_lenght_param_spline->InterpolateMthDerivative(0, length),
  ignition::math::Vector3d(20.0, 0.0, 0.0),
  kLinearTolerance);
}

GTEST_TEST(RNDFSplineHelperTest, StraightSplineFindClosesPointTo) {
  std::vector<
    std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));

  std::unique_ptr<ignition::math::Spline> spline =
    CreateSpline(control_points);
  std::unique_ptr<ArcLengthParameterizedSpline> arc_lenght_param_spline =
    std::make_unique<ArcLengthParameterizedSpline>(
      std::move(spline), kLinearTolerance);
  // Border checks
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(0.0, 5.0, 0.0), kLinearStep), 0.,
    kLinearTolerance);
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(0.0, -5.0, 0.0), kLinearStep), 0.,
    kLinearTolerance);
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(20.0, 5.0, 0.0), kLinearStep), 20.,
    kLinearTolerance);
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(20.0, -5.0, 0.0), kLinearStep), 20.,
    kLinearTolerance);
  // Middle checks
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(10.0, 5.0, 0.0), kLinearStep), 10.,
    kLinearTolerance);
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(10.0, -5.0, 0.0), kLinearStep), 10.,
    kLinearTolerance);
  // Before and after checks
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(-5, -5.0, 0.0), kLinearStep), 0.,
    kLinearTolerance);
  EXPECT_NEAR(arc_lenght_param_spline->FindClosestPointTo(
    ignition::math::Vector3d(25, 5.0, 0.0), kLinearStep), 20.,
    kLinearTolerance);
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
