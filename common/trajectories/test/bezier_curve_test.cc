#include "drake/common/trajectories/bezier_curve.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic/polynomial.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace trajectories {
namespace {

// Line segment from (1,3) to (2,7).
GTEST_TEST(BezierCurveTest, Linear) {
  Eigen::Matrix2d points;
  // clang-format off
  points << 1, 2,
            3, 7;
  // clang-format on

  BezierCurve<double> curve(2, 3, points);

  EXPECT_EQ(curve.order(), 1);
  EXPECT_EQ(curve.start_time(), 2.0);
  EXPECT_EQ(curve.end_time(), 3.0);
  EXPECT_TRUE(
      CompareMatrices(curve.value(2.5), Eigen::Vector2d(1.5, 5), 1e-14));
  EXPECT_TRUE(
      CompareMatrices(curve.value(0), Eigen::Vector2d(1, 3), 1e-14));

  // Extract the symoblic exprssion for the bezier curve.
  const VectorX<symbolic::Expression> curve_expression{
      curve.GetExpression(symbolic::Variable("t"))};
  for (int i = 0; i < curve_expression.rows(); i++) {
    EXPECT_TRUE(curve_expression(i).is_polynomial());
    EXPECT_EQ(symbolic::Polynomial(curve_expression(i)).TotalDegree(), 1);
  }

  auto deriv = curve.MakeDerivative();
  BezierCurve<double>& deriv_bezier =
      dynamic_cast<BezierCurve<double>&>(*deriv);
  EXPECT_EQ(deriv_bezier.order(), 0);
  EXPECT_EQ(deriv->start_time(), 2.0);
  EXPECT_EQ(deriv->end_time(), 3.0);
  EXPECT_TRUE(CompareMatrices(deriv->value(2.5), Eigen::Vector2d(1, 4), 1e-14));

  EXPECT_TRUE(
      CompareMatrices(curve.EvalDerivative(2.5), Eigen::Vector2d(1, 4), 1e-14));
}

// Quadratic curve: [ (1-t)²; t^2 ]
GTEST_TEST(BezierCurveTest, Quadratic) {
  Eigen::Matrix<double, 2, 3> points;
  // clang-format off
  points << 1, 0, 0,
            0, 0, 1;
  // clang-format on
  BezierCurve<double> curve(0, 1, points);

  EXPECT_EQ(curve.order(), 2);
  EXPECT_EQ(curve.start_time(), 0);
  EXPECT_EQ(curve.end_time(), 1.0);

  // derivative is [ -2(1-t); 2t ]
  auto deriv = curve.MakeDerivative();
  BezierCurve<double>& deriv_bezier =
      dynamic_cast<BezierCurve<double>&>(*deriv);
  EXPECT_EQ(deriv_bezier.order(), 1);
  EXPECT_EQ(deriv->start_time(), 0.0);
  EXPECT_EQ(deriv->end_time(), 1.0);

  // second derivative is [ 2; 2 ]
  auto second_deriv = curve.MakeDerivative(2);
  BezierCurve<double>& second_deriv_bezier =
      dynamic_cast<BezierCurve<double>&>(*second_deriv);
  EXPECT_EQ(second_deriv_bezier.order(), 0);
  EXPECT_EQ(second_deriv->start_time(), 0.0);
  EXPECT_EQ(second_deriv->end_time(), 1.0);

  auto second_deriv_via_deriv = deriv->MakeDerivative();
  BezierCurve<double>& second_deriv_via_deriv_bezier =
      dynamic_cast<BezierCurve<double>&>(*second_deriv_via_deriv);
  // second derivative matches, if obtained by differentiating twice.
  EXPECT_TRUE(CompareMatrices(second_deriv_bezier.control_points(),
                              second_deriv_via_deriv_bezier.control_points()));

  // Note: includes evaluations outside the start and end times of the curve.
  for (double sample_time = -0.2; sample_time <= 1.2; sample_time += 0.1) {
    const double t = std::clamp(sample_time, 0.0, 1.0);
    EXPECT_TRUE(CompareMatrices(curve.value(sample_time),
                                Eigen::Vector2d((1 - t) * (1 - t), t * t),
                                1e-14));
    EXPECT_TRUE(CompareMatrices(curve.value(sample_time),
                                Eigen::Vector2d((1 - t) * (1 - t), t * t),
                                1e-14));

    EXPECT_TRUE(CompareMatrices(deriv->value(sample_time),
                                Eigen::Vector2d(-2 * (1 - t), 2 * t), 1e-14));
    EXPECT_TRUE(CompareMatrices(curve.EvalDerivative(sample_time),
                                Eigen::Vector2d(-2 * (1 - t), 2 * t), 1e-14));
    EXPECT_TRUE(CompareMatrices(second_deriv->value(sample_time),
                                Eigen::Vector2d(2, 2), 1e-14));
    EXPECT_TRUE(CompareMatrices(curve.EvalDerivative(sample_time, 2),
                                Eigen::Vector2d(2, 2), 1e-14));
  }
  
  // Extract the symoblic exprssion for the bezier curve.
  VectorX<symbolic::Expression> curve_expression{
      curve.GetExpression(symbolic::Variable("t"))};
  for (int i = 0; i < curve_expression.rows(); i++) {
    EXPECT_TRUE(curve_expression(i).is_polynomial());
    EXPECT_EQ(symbolic::Polynomial(curve_expression(i)).TotalDegree(), 2);
  }
}

GTEST_TEST(BezierCurve, Clone) {
  Eigen::Matrix<double, 2, 3> points;
  // clang-format off
  points << 1, 0, 0,
            0, 0, 1;
  // clang-format on
  BezierCurve<double> curve(0, 1, points);

  auto clone = curve.Clone();
  BezierCurve<double>& clone_bezier =
      dynamic_cast<BezierCurve<double>&>(*clone);

  EXPECT_EQ(clone->start_time(), curve.start_time());
  EXPECT_EQ(clone->end_time(), curve.end_time());
  EXPECT_EQ(clone_bezier.control_points(), curve.control_points());
}

GTEST_TEST(BezierCurve, ScalarTypes) {
  Eigen::Matrix<double, 2, 3> points;
  // clang-format off
  points << 1, 0, 0,
            0, 0, 1;
  // clang-format on
  BezierCurve<double> curve(0, 1, points);
  BezierCurve<AutoDiffXd> curve_ad(0, 1, points.cast<AutoDiffXd>());
  BezierCurve<symbolic::Expression> curve_sym(
      0, 1, points.cast<symbolic::Expression>());

  EXPECT_TRUE(CompareMatrices(curve.value(0.5), curve_ad.value(0.5), 1e-14));
  EXPECT_TRUE(CompareMatrices(curve.value(0.5), curve_sym.value(0.5), 1e-14));
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
