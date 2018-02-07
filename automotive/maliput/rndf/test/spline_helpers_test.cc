#include "drake/automotive/maliput/rndf/spline_helpers.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>
#include "ignition/math/Spline.hh"
#include "ignition/math/Vector3.hh"

#include "drake/automotive/maliput/rndf/test_utilities/ignition_types_compare.h"

namespace drake {
namespace maliput {
namespace rndf {
namespace {

// We use this tolerance to match the interpolated positions given by the path
// length interpolator wrappers.
const double kLinearTolerance = 1e-4;
// We use this step to sample the spline at this rate, to get the closest path
// length coordinate to a point in space.
const double kLinearStep = 1e-2;

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
  // xmin is equal to xmax.
  EXPECT_THROW(
      InverseFunctionInterpolator([](double t) { return t; }, 0., 0., -1.),
      std::runtime_error);
  // xmin is greater than xmax.
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
  // Spline pointer set to nullptr.
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
    EXPECT_TRUE(test::IsIgnitionVector3dClose(
        arc_length_param_spline->InterpolateMthDerivative(0, l), p,
        kLinearTolerance));
    EXPECT_TRUE(test::IsIgnitionVector3dClose(
        arc_length_param_spline->InterpolateMthDerivative(1, l),
        kFirstDerivativeInterpolation, kLinearTolerance));
    EXPECT_TRUE(test::IsIgnitionVector3dClose(
        arc_length_param_spline->InterpolateMthDerivative(2, l),
        kSecondDerivativeInterpolation, kLinearTolerance));
  }
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
     arc_length_param_spline->InterpolateMthDerivative(0, length), kEndPoint,
      kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      arc_length_param_spline->InterpolateMthDerivative(1, length),
      kFirstDerivativeInterpolation, kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(
      arc_length_param_spline->InterpolateMthDerivative(2, length),
      kSecondDerivativeInterpolation, kLinearTolerance));
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
  // Border checks.
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
  // Middle checks.
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(10.0, 5.0, 0.0), kLinearStep),
              10., kLinearTolerance);
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(10.0, -5.0, 0.0), kLinearStep),
              10., kLinearTolerance);
  // Before and after checks.
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(-5, -5.0, 0.0), kLinearStep),
              0., kLinearTolerance);
  EXPECT_NEAR(arc_length_param_spline->FindClosestPointTo(
                  ignition::math::Vector3d(25, 5.0, 0.0), kLinearStep),
              20., kLinearTolerance);
}

// Checks the conversion from points and tangents of a Hermite Spline to a
// Cubic Bezier curve.
GTEST_TEST(RNDFSplineToBezierTest, ConversionTest) {
  ignition::math::Vector3d p0, t0, p1, t1;
  // First case where the points and tangents form a 90° shape. Tangents are
  // set so both control points are at the critical point.
  p0.Set(0.0, 10.0, 0.0);
  t0.Set(0.0, -30.0, 0.0);
  p1.Set(10.0, 0.0, 0.0);
  t1.Set(30.0, 0.0, 0.0);
  std::vector<ignition::math::Vector3d> bezier_points =
      SplineToBezier(p0, t0, p1, t1);
  EXPECT_EQ(bezier_points.size(), 4);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[1],
                                            ignition::math::Vector3d::Zero,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[2],
                                            ignition::math::Vector3d::Zero,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[3],
                                            p1,
                                            kLinearTolerance));

  // Second case sets points under the critical curvature.
  p0.Set(0.0, 10.0, 0.0);
  t0.Set(0.0, -10.0, 0.0);
  p1.Set(10.0, 0.0, 0.0);
  t1.Set(10.0, 0.0, 0.0);
  bezier_points = SplineToBezier(p0, t0, p1, t1);
  EXPECT_EQ(bezier_points.size(), 4);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[1],
                                            ignition::math::Vector3d(0.0,
                                                2.0 * 10.0 / 3., 0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[2],
                                            ignition::math::Vector3d(
                                                2.0 * 10.0 / 3.,
                                                0.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[3],
                                            p1,
                                            kLinearTolerance));

  // Third case sets points above the critical curvature.
  p0.Set(0.0, 10.0, 0.0);
  t0.Set(0.0, -60.0, 0.0);
  p1.Set(10.0, 0.0, 0.0);
  t1.Set(60.0, 0.0, 0.0);
  bezier_points = SplineToBezier(p0, t0, p1, t1);
  EXPECT_EQ(bezier_points.size(), 4);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[1],
                                            ignition::math::Vector3d(0.0,
                                                -10.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[2],
                                            ignition::math::Vector3d(-10.0,
                                                0.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(bezier_points[3],
                                            p1,
                                            kLinearTolerance));
}

// Checks the conversion from control points of a Cubic Bezier curve to a
// set of points and tangents in Hermite Spline base.
GTEST_TEST(RNDFBezierToBezierTest, ConversionTest) {
  ignition::math::Vector3d p0, p1, p2, p3;
  // First case where the control points form a 90° shape. Control points are
  // set to get critical tangents in Hermite base.
  p0.Set(0.0, 10.0, 0.0);
  p1.Set(0.0, 0.0, 0.0);
  p2.Set(0.0, 0.0, 0.0);
  p3.Set(10.0, 0.0, 0.0);
  std::vector<ignition::math::Vector3d> hermite_points =
      BezierToSpline(p0, p1, p2, p3);
  EXPECT_EQ(hermite_points.size(), 4);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[1],
                                            ignition::math::Vector3d(0.0,
                                                -30.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[2],
                                            p3,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[3],
                                            ignition::math::Vector3d(30.0,
                                                0.0,
                                                0.0),
                                            kLinearTolerance));

  // Second case sets points under the critical curvature.
  p0.Set(0.0, 10.0, 0.0);
  p1.Set(0.0, 2.0 * 10.0 / 3., 0.0);
  p2.Set(2.0 * 10.0 / 3., 0.0, 0.0);
  p3.Set(10.0, 0.0, 0.0);
  hermite_points = BezierToSpline(p0, p1, p2, p3);
  EXPECT_EQ(hermite_points.size(), 4);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[1],
                                            ignition::math::Vector3d(0.0,
                                                -10.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[2],
                                            p3,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[3],
                                            ignition::math::Vector3d(10.0,
                                                0.0,
                                                0.0),
                                            kLinearTolerance));

  // Third case sets points above the critical curvature.
  p0.Set(0.0, 10.0, 0.0);
  p1.Set(0.0, -10.0, 0.0);
  p2.Set(-10., 0.0, 0.0);
  p3.Set(10.0, 0.0, 0.0);
  hermite_points = BezierToSpline(p0, p1, p2, p3);
  EXPECT_EQ(hermite_points.size(), 4);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[1],
                                            ignition::math::Vector3d(0.0,
                                                -60.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[2],
                                            p3,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(hermite_points[3],
                                            ignition::math::Vector3d(60.0,
                                                0.0,
                                                0.0),
                                            kLinearTolerance));
}

// Checks that all the constraints throw when conditions are not satisfied.
GTEST_TEST(RNDFMakeBezierCurveMonotonicTest, ExceptionCases) {
  const ignition::math::Vector3d p0(0.0, 10.0, 0.0);
  const ignition::math::Vector3d p1(0.0, 5.0, 0.0);
  const ignition::math::Vector3d p2(5.0, 0.0, 0.0);
  const ignition::math::Vector3d p3(10.0, 0.0, 0.0);
  std::vector<ignition::math::Vector3d> input_bezier_points = {p0, p1, p2, p3};
  EXPECT_THROW(MakeBezierCurveMonotonic({}, 1.0), std::runtime_error);
  EXPECT_THROW(MakeBezierCurveMonotonic(input_bezier_points, -1.0),
               std::runtime_error);
  EXPECT_THROW(MakeBezierCurveMonotonic(input_bezier_points, 2.0),
               std::runtime_error);
}

// Desired geometry from points and tangents in Hermite base.
// XXX----------->
//     XXXX
//         XXX
//            XXX
//              XXXX
//                 XX
//                  XX
//                   X
//                   X
//                   |
//                   |
//                   |
//                   |
//                   |
//                   v
GTEST_TEST(RNDFMakeBezierCurveMonotonicTest, Case90DegreeConnection) {
  const ignition::math::Vector3d p0(0.0, 10.0, 0.0);
  const ignition::math::Vector3d p1(0.0, 5.0, 0.0);
  const ignition::math::Vector3d p2(5.0, 0.0, 0.0);
  const ignition::math::Vector3d p3(10.0, 0.0, 0.0);
  const std::vector<ignition::math::Vector3d> input_bezier_points = {p0, p1, p2,
                                                                     p3};
  const std::vector<ignition::math::Vector3d> output_bezier_points =
      MakeBezierCurveMonotonic(input_bezier_points, 1.0);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[1],
                                            ignition::math::Vector3d::Zero,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[2],
                                            ignition::math::Vector3d::Zero,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[3],
                                            p3,
                                            kLinearTolerance));
}

// Desired geometry from points and tangents in Hermite base.
//                 XXXXXXXXXXXX-------------------->
//               XX
//               X
//             XXX
//           XXX
//      XXXXXX
// XXXXXX-------------->
GTEST_TEST(RNDFMakeBezierCurveMonotonicTest,
    CaseParallelNonColinearConnection) {
  const ignition::math::Vector3d p0(0.0, 0.0, 0.0);
  const ignition::math::Vector3d p1(20.0, 0.0, 0.0);
  const ignition::math::Vector3d p2(0.0, 10.0, 0.0);
  const ignition::math::Vector3d p3(20.0, 10.0, 0.0);
  const std::vector<ignition::math::Vector3d> input_bezier_points = {p0, p1, p2,
                                                                     p3};
  const std::vector<ignition::math::Vector3d> output_bezier_points =
      MakeBezierCurveMonotonic(input_bezier_points, 1.0);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[1],
                                            ignition::math::Vector3d(10.0,
                                                0.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[2],
                                            ignition::math::Vector3d(10.0,
                                                10.0,
                                                0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[3],
                                            p3,
                                            kLinearTolerance));
}

// Desired geometry from points and tangents in Hermite base.
// +--------------->XXXXXXXX XXXXX+------------->
GTEST_TEST(RNDFMakeBezierCurveMonotonicTest, CaseParallelColinearConnection) {
  const ignition::math::Vector3d p0(0.0, 0.0, 0.0);
  const ignition::math::Vector3d p1(20.0, 0.0, 0.0);
  const ignition::math::Vector3d p2(0.0, 0.0, 0.0);
  const ignition::math::Vector3d p3(20.0, 0.0, 0.0);
  const std::vector<ignition::math::Vector3d> input_bezier_points = {p0, p1, p2,
                                                                     p3};
  const std::vector<ignition::math::Vector3d> output_bezier_points =
      MakeBezierCurveMonotonic(input_bezier_points, 1.0);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[1],
                                            ignition::math::Vector3d(10.0,
                                              0.0,
                                              0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[2],
                                            ignition::math::Vector3d(10.0,
                                              0.0,
                                              0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[3],
                                            p3,
                                            kLinearTolerance));
}

// Desired geometry from points and tangents in Hermite base.
// XXX------->
//   XXXXXX
//        XXX
//           XXX
//             XX
//              X
//               X
//               \X
//                \X
//                 \X
//                  >
GTEST_TEST(RNDFMakeBezierCurveMonotonicTest, CaseObliqueConnection) {
  const ignition::math::Vector3d p0(0.0, 10.0, 0.0);
  const ignition::math::Vector3d p1(10.0, 10.0, 0.0);
  const ignition::math::Vector3d p2(10.0, 5.0, 0.0);
  const ignition::math::Vector3d p3(15.0, 0.0, 0.0);
  const std::vector<ignition::math::Vector3d> input_bezier_points = {p0, p1, p2,
                                                                     p3};
  const std::vector<ignition::math::Vector3d> output_bezier_points =
      MakeBezierCurveMonotonic(input_bezier_points, 1.0);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[2],
                                            ignition::math::Vector3d(5.0,
                                              10.0,
                                              0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[3],
                                            p3,
                                            kLinearTolerance));
}

// Desired geometry from points and tangents in Hermite base.
//     +
//     XX\X
//       X\X
//         \XX
//          >X
//          XX
//        XXX
//    XXXX
// XXXX
// XX
// X
// XX
// XXXXX+------>
GTEST_TEST(RNDFMakeBezierCurveMonotonicTest, CaseObliqueNonConvexConnection) {
  const ignition::math::Vector3d p0(0.0, 10.0, 0.0);
  const ignition::math::Vector3d p1(5.0, 5.0, 0.0);
  const ignition::math::Vector3d p2(-5.0, 0.0, 0.0);
  const ignition::math::Vector3d p3(0.0, 0.0, 0.0);
  const std::vector<ignition::math::Vector3d> input_bezier_points = {p0, p1, p2,
                                                                     p3};
  const std::vector<ignition::math::Vector3d> output_bezier_points =
      MakeBezierCurveMonotonic(input_bezier_points, 0.5);
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[0],
                                            p0,
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[1],
                                            ignition::math::Vector3d(0.5,
                                              9.5,
                                              0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[2],
                                            ignition::math::Vector3d(-0.5,
                                              0.0,
                                              0.0),
                                            kLinearTolerance));
  EXPECT_TRUE(test::IsIgnitionVector3dClose(output_bezier_points[3],
                                            p3,
                                            kLinearTolerance));
}

// Checks that all the constraints throw when conditions are not satisfied.
GTEST_TEST(RNDFCreatePChipBasedSplineTest, CaseExceptions) {
  ::testing::FLAGS_gtest_death_test_style = "fast";
  // Checks that it throws when a bad vector is supplied.
  EXPECT_THROW(CreatePChipBasedSpline({}), std::runtime_error);
  // Check the nice case.
  std::vector<ignition::math::Vector3d> points;
  points.push_back(ignition::math::Vector3d(0.0, 0.0, 0.0));
  points.push_back(ignition::math::Vector3d(10.0, 10.0, 0.0));
  points.push_back(ignition::math::Vector3d(20.0, 0.0, 0.0));
  EXPECT_NE(CreatePChipBasedSpline(points), nullptr);
  // Checks that it throws when two consecutive points are the same.
  points.push_back(ignition::math::Vector3d(20.0, 0.0, 0.0));
  EXPECT_THROW(CreatePChipBasedSpline(points), std::runtime_error);
}

}  // namespace
}  // namespace rndf
}  // namespace maliput
}  // namespace drake
