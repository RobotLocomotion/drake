#include "drake/common/trajectories/bezier_curve.h"

#include <memory>
#include <utility>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic/polynomial.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace trajectories {
namespace {

using Eigen::Map;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Vector2d;
using Eigen::VectorXd;

// Tests the default constructor.
GTEST_TEST(BezierCurveTest, DefaultConstructor) {
  BezierCurve<double> curve;
  EXPECT_EQ(curve.order(), -1);
  EXPECT_EQ(curve.start_time(), 0);
  EXPECT_EQ(curve.end_time(), 1);
  EXPECT_EQ(curve.rows(), 0);
  EXPECT_EQ(curve.cols(), 1);

  EXPECT_EQ(curve.BernsteinBasis(0, 0), 0);
  EXPECT_EQ(curve.control_points().size(), 0);
  EXPECT_EQ(curve.Clone()->rows(), 0);
  EXPECT_EQ(curve.value(0).size(), 0);
  EXPECT_EQ(curve.GetExpression().size(), 0);
  curve.ElevateOrder();
  EXPECT_EQ(curve.order(), 0);
  EXPECT_EQ(curve.control_points().size(), 0);
}

GTEST_TEST(BezierCurveTest, Constant) {
  const Eigen::Vector2d kValue(1, 2);
  BezierCurve<double> curve(0, 1, kValue);
  EXPECT_EQ(curve.order(), 0);
  EXPECT_EQ(curve.start_time(), 0);
  EXPECT_EQ(curve.end_time(), 1);
  EXPECT_EQ(curve.rows(), 2);
  EXPECT_EQ(curve.cols(), 1);
  EXPECT_TRUE(CompareMatrices(curve.value(0.5), kValue));

  auto M = curve.AsLinearInControlPoints(1);
  EXPECT_EQ(M.rows(), 1);
  EXPECT_EQ(M.cols(), 0);

  curve.ElevateOrder();
  EXPECT_EQ(curve.order(), 1);
  EXPECT_TRUE(CompareMatrices(curve.value(0.5), kValue));
}

using drake::geometry::optimization::VPolytope;
void CheckConvexHullProperty(const BezierCurve<double>& curve,
                             int num_samples) {
  VPolytope control_polytope{curve.control_points()};
  const double tol_sol = 1e-7;
  Eigen::VectorXd samples = Eigen::VectorXd::LinSpaced(
      num_samples, curve.start_time(), curve.end_time());
  for (int i = 0; i < num_samples; ++i) {
    EXPECT_TRUE(control_polytope.PointInSet(curve.value(samples(i)), tol_sol));
  }
}

// An empty curve.
GTEST_TEST(BezierCurveTest, Default) {
  BezierCurve<double> curve;
  EXPECT_EQ(curve.order(), -1);
  EXPECT_EQ(curve.control_points().rows(), 0);
  EXPECT_EQ(curve.control_points().cols(), 0);
  EXPECT_NO_THROW(curve.Clone());
}

// A moved-from curve.
GTEST_TEST(BezierCurveTest, MoveConstructor) {
  Eigen::Matrix2d points;
  // clang-format off
  points << 1, 2,
            3, 7;
  // clang-format on
  BezierCurve<double> orig(2, 3, points);
  EXPECT_EQ(orig.order(), 1);
  EXPECT_EQ(orig.start_time(), 2.0);
  EXPECT_EQ(orig.end_time(), 3.0);

  // A moved-into trajectory retains the original information.
  BezierCurve<double> dest(std::move(orig));
  EXPECT_EQ(dest.order(), 1);
  EXPECT_EQ(dest.start_time(), 2.0);
  EXPECT_EQ(dest.end_time(), 3.0);

  // The moved-from trajectory must still be in a valid state.
  EXPECT_EQ(orig.order(), orig.control_points().cols() - 1);
  EXPECT_GE(orig.end_time(), orig.start_time());
}

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

  const int num_samples{100};
  CheckConvexHullProperty(curve, num_samples);
  CheckConvexHullProperty(deriv_bezier, num_samples);

  auto M = curve.AsLinearInControlPoints(1);
  EXPECT_EQ(M.rows(), 2);
  EXPECT_EQ(M.cols(), 1);
  EXPECT_TRUE(CompareMatrices(deriv_bezier.control_points(),
                              curve.control_points() * M, 1e-14));
  M = curve.AsLinearInControlPoints(0);
  EXPECT_TRUE(CompareMatrices(M.toDense(), Matrix2d::Identity()));
  M = curve.AsLinearInControlPoints(2);
  EXPECT_EQ(M.rows(), 2);
  EXPECT_EQ(M.cols(), 0);

  curve.ElevateOrder();
  EXPECT_EQ(curve.order(), 2);
  EXPECT_TRUE(
      CompareMatrices(curve.value(2.5), Eigen::Vector2d(1.5, 5), 1e-14));
  EXPECT_TRUE(CompareMatrices(curve.value(0), Eigen::Vector2d(1, 3), 1e-14));
  EXPECT_TRUE(CompareMatrices(curve.value(3.0), Eigen::Vector2d(2, 7), 1e-14));
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

  auto M = curve.AsLinearInControlPoints(1);
  EXPECT_EQ(M.rows(), 3);
  EXPECT_EQ(M.cols(), 2);
  EXPECT_TRUE(CompareMatrices(deriv_bezier.control_points(),
                              curve.control_points() * M, 1e-14));

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
    EXPECT_TRUE(CompareMatrices(deriv->value(sample_time),
                                Eigen::Vector2d(-2 * (1 - t), 2 * t), 1e-14));
    EXPECT_TRUE(CompareMatrices(curve.EvalDerivative(sample_time),
                                Eigen::Vector2d(-2 * (1 - t), 2 * t), 1e-14));
    EXPECT_TRUE(CompareMatrices(second_deriv->value(sample_time),
                                Eigen::Vector2d(2, 2), 1e-14));
    EXPECT_TRUE(CompareMatrices(curve.EvalDerivative(sample_time, 2),
                                Eigen::Vector2d(2, 2), 1e-14));
  }

  M = curve.AsLinearInControlPoints(2);
  EXPECT_EQ(M.rows(), 3);
  EXPECT_EQ(M.cols(), 1);
  EXPECT_TRUE(CompareMatrices(second_deriv_bezier.control_points(),
                              curve.control_points() * M, 1e-14));

  // Extract the symbolic expression for the bezier curve.
  VectorX<symbolic::Expression> curve_expression{
      curve.GetExpression(symbolic::Variable("t"))};
  for (int i = 0; i < curve_expression.rows(); i++) {
    EXPECT_TRUE(curve_expression(i).is_polynomial());
    EXPECT_EQ(symbolic::Polynomial(curve_expression(i)).TotalDegree(), 2);
  }

  const int num_samples{100};
  CheckConvexHullProperty(curve, num_samples);
  CheckConvexHullProperty(deriv_bezier, num_samples);

  curve.ElevateOrder();
  EXPECT_EQ(curve.order(), 3);
  for (double sample_time = -0.2; sample_time <= 1.2; sample_time += 0.1) {
    const double t = std::clamp(sample_time, 0.0, 1.0);
    EXPECT_TRUE(CompareMatrices(curve.value(sample_time),
                                Eigen::Vector2d((1 - t) * (1 - t), t * t),
                                1e-14));
  }
}

// Test the example from the AsLinearInControlPoints doc string.
GTEST_TEST(BezierCurve, ConstraintDerivativeControlPoint) {
  Eigen::Matrix<double, 2, 3> points;
  // clang-format off
  points << 1, 0, 0,
            0, 0, 1;
  // clang-format on
  BezierCurve<double> curve(0, 1, points);
  auto deriv = curve.MakeDerivative();
  BezierCurve<double>& deriv_bezier =
      dynamic_cast<BezierCurve<double>&>(*deriv);

  solvers::MathematicalProgram prog;
  auto p = prog.NewContinuousVariables<2, 3>("p");
  prog.SetInitialGuess(p, points);

  SparseMatrix<double> M = curve.AsLinearInControlPoints(1);
  const Vector2d lb(-0.1, -0.2);
  const Vector2d ub(0.1, 0.2);
  const int k = 1;
  // Test the code sample.
  for (int i = 0; i < curve.rows(); ++i) {
    auto constraint = std::make_shared<solvers::LinearConstraint>(
        M.col(k).transpose(), Vector1d(lb(i)), Vector1d(ub(i)));
    auto b = prog.AddConstraint(constraint, p.row(i).transpose());
    EXPECT_NEAR(prog.EvalBindingAtInitialGuess(b)[0],
                deriv_bezier.control_points()(i, k), 1e-12);
  }

  // TODO(russt): Implement blkdiag for SparseMatrix and switch this example to
  // use it instead of converting to dense matrices.

  /* Test the block matrix documentation:
   vec(derivative.control_points().T) = blockMT * vec(this.control_points().T),
   blockMT = [ M.T,   0, .... 0 ]
             [   0, M.T, 0, ... ]
             [      ...         ]
             [  ...    , 0, M.T ].
  */
  const int m = M.rows();
  const int n = M.cols();
  MatrixXd blockMT = MatrixXd::Zero(n * curve.rows(), m * curve.rows());
  for (int i = 0; i < curve.rows(); ++i) {
    blockMT.block(i * n, i * m, n, m) = M.transpose().toDense();
  }
  Map<const VectorXd> vec_curve_points(
      curve.control_points().transpose().data(),
      curve.rows() * (curve.order() + 1));
  Map<const VectorXd> vec_deriv_points(
      deriv_bezier.control_points().transpose().data(),
      curve.rows() * (deriv_bezier.order() + 1));
  EXPECT_TRUE(
      CompareMatrices(vec_deriv_points, blockMT * vec_curve_points, 1e-12));
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
  // degree and that all scalar types return the same expression when the
  // underlying control points are the same. Whether the underlying expression
  // is correct must be done separately.
  auto test_expression_from_points =
      [&t](double start_time, double end_time,
           const Eigen::Ref<const Eigen::MatrixXd>& points) {
        BezierCurve<double> curve_double(start_time, end_time, points);
        BezierCurve<AutoDiffXd> curve_ad(start_time, end_time,
                                         points.cast<AutoDiffXd>());
        BezierCurve<symbolic::Expression> curve_sym(
            start_time, end_time, points.cast<symbolic::Expression>());

        const VectorX<symbolic::Expression> curve_expression{
            curve_sym.GetExpression(t)};
        const VectorX<symbolic::Expression> curve_expression_double{
            curve_double.GetExpression(t)};
        const VectorX<symbolic::Expression> curve_expression_ad{
            curve_ad.GetExpression(t)};
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
