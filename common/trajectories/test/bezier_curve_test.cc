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
  EXPECT_TRUE(CompareMatrices(curve.value(0), Eigen::Vector2d(1, 3), 1e-14));

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

GTEST_TEST(BezierCurve, GetExpressionLinear) {
  symbolic::Variable t{"t"};

  // Tests that the call to GetExpression returns a polynomial of appropriate
  // degree and that all scalar types return the same expression. Whether the
  // underlying expression is correct must be done separately.
  auto test_expression_from_points =
      [&t](double start_time, double end_time,
           const Eigen::Ref<const Eigen::MatrixXd>& points) {
        BezierCurve<double> curve(start_time, end_time, points);
        BezierCurve<AutoDiffXd> curve_ad(start_time, end_time,
                                         points.cast<AutoDiffXd>());
        BezierCurve<symbolic::Expression> curve_sym(
            start_time, end_time, points.cast<symbolic::Expression>());

        const VectorX<symbolic::Expression> curve_expression{
            curve_sym.GetExpression(t)};
        const VectorX<symbolic::Expression> curve_expression_double{
            curve_sym.GetExpression(t)};
        const VectorX<symbolic::Expression> curve_expression_ad{
            curve_sym.GetExpression(t)};
        for (int i = 0; i < curve_expression.rows(); i++) {
          EXPECT_TRUE(curve_expression(i).is_polynomial());
          EXPECT_EQ(symbolic::Polynomial(curve_expression(i)).TotalDegree(),
                    points.cols() - 1);
          EXPECT_TRUE(curve_expression(i).EqualTo(curve_expression_double(i)));
          EXPECT_TRUE(curve_expression(i).EqualTo(curve_expression_ad(i)));
        }
        return curve_expression;
      };

  // Line segment from (1,3) to (2,7).
  Eigen::Matrix2d points_linear;
  // clang-format off
  points_linear << 1, 2,
                   3, 7;
  // clang-format on
  const double start_time_linear{2};
  const double end_time_linear{3};
  const VectorX<symbolic::Expression> linear_curve_expression =
      test_expression_from_points(start_time_linear, end_time_linear,
                                  points_linear);
  for (int i = 0; i < linear_curve_expression.rows(); i++) {
    symbolic::Expression expr_expected =
        (end_time_linear - t) * points_linear(i, 0) +
        (t - start_time_linear) * points_linear(i, 1);
    EXPECT_TRUE(linear_curve_expression(i).EqualTo(expr_expected.Expand()));
  }

  // Quadratic curve: [ (1-t)²; t^2 ]
  Eigen::Matrix<double, 2, 3> points_quadratic;
  // clang-format off
  points_quadratic << 1, 0, 0,
                      0, 0, 1;
  // clang-format on
  const VectorX<symbolic::Expression> quadratic_curve_expression =
      test_expression_from_points(0, 1, points_quadratic);
  EXPECT_TRUE(quadratic_curve_expression(0).EqualTo(pow(t, 2) - 2 * t + 1));
  EXPECT_TRUE(quadratic_curve_expression(1).EqualTo(pow(t, 2)));
}

}  // namespace
}  // namespace trajectories
}  // namespace drake
