#include "drake/solvers/constraint.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace solvers {

using symbolic::Variable;
using symbolic::Expression;

namespace {
GTEST_TEST(testConstraint, testLinearConstraintUpdate) {
  // Update the coefficients or the bound of the linear constraint, and check
  // the updated constraint.
  const Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  const Eigen::Vector2d b(1, 2);
  LinearEqualityConstraint constraint(A, b);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), b));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), b));
  EXPECT_TRUE(CompareMatrices(constraint.A(), A));
  EXPECT_EQ(constraint.num_constraints(), 2);

  // Update with a new matrix A2 with three columns. This should cause a runtime
  // error, since the number of variables do not match.
  const Eigen::Matrix<double, 2, 3> A2 = Eigen::Matrix<double, 2, 3>::Ones();
  const Eigen::Vector2d b2(1, 2);
  EXPECT_THROW(constraint.UpdateCoefficients(A2, b2), std::runtime_error);

  // Update with a new matrix A3 with size 3 x 2.
  const Eigen::Matrix<double, 3, 2> A3 = Eigen::Matrix<double, 3, 2>::Ones();
  const Eigen::Vector3d b3(1, 2, 3);
  constraint.UpdateCoefficients(A3, b3);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), b3));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), b3));
  EXPECT_TRUE(CompareMatrices(constraint.A(), A3));
  EXPECT_EQ(constraint.num_constraints(), 3);
}
GTEST_TEST(testConstraint, testQuadraticConstraintHessian) {
  // Check if the getters in the QuadraticConstraint are right.
  Eigen::Matrix2d Q;
  Eigen::Vector2d b;
  // clang-format off
  Q << 1, 0,
       0, 1;
  // clang-format on
  b << 1, 2;
  // Constructs a constraint with a symmetric Q.
  QuadraticConstraint constraint1(Q, b, 0, 1);
  EXPECT_TRUE(CompareMatrices(constraint1.Q(), Q));
  EXPECT_TRUE(CompareMatrices(constraint1.b(), b));

  // Updates constraint with a non-symmetric Hessian.
  // clang-format off
  Q << 1, 1,
       0, 1;
  // clang-format on
  b << 1, 2;
  constraint1.UpdateCoefficients(Q, b);
  EXPECT_TRUE(CompareMatrices(constraint1.Q(), (Q + Q.transpose()) / 2));
  EXPECT_TRUE(CompareMatrices(constraint1.b(), b));

  // Constructs a constraint with a non-symmetric Hessian.
  QuadraticConstraint constraint2(Q, b, 0, 1);
  EXPECT_TRUE(CompareMatrices(constraint2.Q(), (Q + Q.transpose()) / 2));
  EXPECT_TRUE(CompareMatrices(constraint2.b(), b));
}

// Tests if the Lorentz Cone constraint is imposed correctly.
void TestLorentzConeEval(const Eigen::Ref<const Eigen::MatrixXd> A,
                         const Eigen::Ref<const Eigen::VectorXd> b,
                         const VectorXd& x_test, bool is_in_cone) {
  LorentzConeConstraint cnstr(A, b);
  VectorXd y;
  // Test Eval with VectorXd.
  cnstr.Eval(x_test, y);
  Vector2d y_expected;
  VectorXd z = A * x_test + b;
  y_expected(0) = z(0);
  y_expected(1) = z(0) * z(0) - z.tail(z.size() - 1).squaredNorm();
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected = (y(0) >= 0) & (y(1) >= 0);
  EXPECT_EQ(is_in_cone, is_in_cone_expected);
  EXPECT_EQ(cnstr.CheckSatisfied(x_test), is_in_cone_expected);

  auto tx = drake::math::initializeAutoDiff(x_test);
  AutoDiffVecXd x_taylor = tx;
  AutoDiffVecXd y_taylor;
  // Test Eval with AutoDiff.
  cnstr.Eval(x_taylor, y_taylor);

  EXPECT_TRUE(CompareMatrices(y, math::autoDiffToValueMatrix(y_taylor)));
  EXPECT_EQ(cnstr.CheckSatisfied(x_taylor), is_in_cone_expected);
}

void TestRotatedLorentzConeEval(const Eigen::Ref<const Eigen::MatrixXd> A,
                                const Eigen::Ref<const Eigen::VectorXd> b,
                                const VectorXd& x_test, bool is_in_cone) {
  RotatedLorentzConeConstraint cnstr(A, b);
  VectorXd y;
  cnstr.Eval(x_test, y);
  Eigen::VectorXd z = A * x_test + b;
  Vector3d y_expected(
      z(0),
      z(1),
      z(0) * z(1) - z.tail(z.size() - 2).squaredNorm());
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected =
      (z(0) >= 0) & (z(1) >= 0) & (z(0) * z(1) >= z.tail(z.size() - 2).norm());
  EXPECT_EQ(is_in_cone, is_in_cone_expected);
  EXPECT_EQ(cnstr.CheckSatisfied(x_test), is_in_cone_expected);

  // Eval with taylor var.
  auto tx = drake::math::initializeAutoDiff(x_test);
  AutoDiffVecXd x_taylor = tx;
  AutoDiffVecXd y_taylor;
  cnstr.Eval(x_taylor, y_taylor);

  EXPECT_TRUE(CompareMatrices(y, math::autoDiffToValueMatrix(y_taylor)));
  EXPECT_EQ(cnstr.CheckSatisfied(x_taylor), is_in_cone_expected);
}

GTEST_TEST(testConstraint, testLorentzConeConstraint) {
  // [3;1;1] is in the interior of the Lorentz cone.
  Eigen::Vector3d x1(3.0, 1.0, 1.0);
  TestLorentzConeEval(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), x1,
                      true);

  // [3;2;2;1] is on the boundary of the Lorentz cone.
  Eigen::Vector2d x2(1, 3);
  Eigen::Matrix<double, 4, 2> A2;
  // clang-format off
  A2 << 1, 0,
       1, 1,
       -1, 1,
       1, -2;
  // clang-format on
  Eigen::Vector4d b2(2, -2, 0, 6);
  TestLorentzConeEval(A2, b2, x2, true);

  // [3; 3; 1] is outside of the Lorentz cone.
  Eigen::Vector4d x3(1, -1, 2, 3);
  Eigen::Matrix<double, 3, 4> A3;
  // clang-format off
  A3 << 1, 0, -1, 2,
        -1, 2, 0, 1,
        0, -2, 3, 1;
  // clang-format on
  Eigen::Vector3d b3 = Eigen::Vector3d(3, 3, 1) - A3 * x3;
  TestLorentzConeEval(A3, b3, x3, false);

  // [-3; 1; 1] is outside of the Lorentz cone.
  Vector1d x4 = Vector1d::Constant(4);
  Eigen::Vector3d A4(-1, 3, 2);
  Eigen::Vector3d b4 = Eigen::Vector3d(-3, 1, 1) - A4 * x4;
  TestLorentzConeEval(A4, b4, x4, false);
}

GTEST_TEST(testConstraint, testRotatedLorentzConeConstraint) {
  // [1;2;1] is in the interior of the rotated lorentz cone.
  TestRotatedLorentzConeEval(Eigen::Matrix3d::Identity(),
                             Eigen::Vector3d::Zero(), Vector3d(1, 2, 1), true);

  // [1;2;1;1] is on the boundary of the rotated Lorentz cone.
  Eigen::Vector2d x2(1, 2);
  Eigen::Matrix<double, 4, 2> A2;
  // clang-format off
  A2 << 1, -1,
        0, 2,
        -1, 3,
        -2, 4;
  // clang-format on
  Eigen::Vector4d b2 = Eigen::Vector4d(1, 2, 1, 1) - A2 * x2;
  TestRotatedLorentzConeEval(A2, b2, x2, true);

  // [1;2;2;2] is outside of the rotated Lorentz cone.
  Eigen::Vector4d x3(1, 3, -1, 2);
  Eigen::Matrix4d A3;
  // clang-format off
  A3 << 1, 2, 3, 4,
        -1, 2, 4, 2,
        -3, 2, 1, 4,
        2, 1, 3, 2;
  // clang-format on
  Eigen::Vector4d b3 = Eigen::Vector4d(1, 2, 2, 2) - A3 * x3;
  TestRotatedLorentzConeEval(A3, b3, x3, false);

  // [-1; -2; 1] is outside of the rotated Lorentz cone.
  Vector1d x4 = Vector1d::Constant(10);
  Eigen::Vector3d A4(1, 3, 2);
  Eigen::Vector3d b4 = Eigen::Vector3d(-1, -2, 1) - A4 * x4;
  TestRotatedLorentzConeEval(A4, b4, x4, false);
}

GTEST_TEST(testConstraint, testPositiveSemidefiniteConstraint) {
  PositiveSemidefiniteConstraint cnstr(3);

  Eigen::Matrix<double, 9, 1> X1;
  // clang-format off
  X1 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
  // clang-format on
  Eigen::VectorXd y;
  cnstr.Eval(X1, y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());
  EXPECT_TRUE(cnstr.CheckSatisfied(X1));

  Eigen::Matrix<double, 9, 1> X2;
  // clang-format off
  X2 << 1, 2, 0,
        2, -2, -1,
        0, -1, -2;
  // clang-format on
  cnstr.Eval(X2, y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
  EXPECT_EQ(cnstr.matrix_rows(), 3);
  EXPECT_FALSE(cnstr.CheckSatisfied(X2));
}

GTEST_TEST(testConstraint, testLinearMatrixInequalityConstraint) {
  Eigen::Matrix2d F0 = 2 * Eigen::Matrix2d::Identity();
  Eigen::Matrix2d F1;
  F1 << 1, 1, 1, 1;
  Eigen::Matrix2d F2;
  F2 << 1, 2, 2, 1;
  LinearMatrixInequalityConstraint cnstr({F0, F1, F2});

  // [4, 3]
  // [3, 4] is positive semidefinite
  Eigen::VectorXd y;
  Eigen::Vector2d x1(1, 1);
  cnstr.Eval(x1, y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());
  EXPECT_TRUE(cnstr.CheckSatisfied(x1));

  // [1 -2]
  // [-2 1] is not p.s.d
  Eigen::Vector2d x2(0, -1);
  cnstr.Eval(x2, y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
  EXPECT_FALSE(cnstr.CheckSatisfied(x2));
}

GTEST_TEST(testConstraint, testExpressionConstraint) {
  Variable x0{"x0"};
  Variable x1{"x1"};
  Variable x2{"x2"};

  Vector3<Variable> vars{x0, x1, x2};
  Vector2<Expression> e{ 1. + x0*x0, x1*x1 + x2 };

  ExpressionConstraint constraint(e, Vector2d::Zero(), 2.*Vector2d::Ones());

  const VectorX<symbolic::Expression>& expressions{constraint.expressions()};
  ASSERT_EQ(expressions.size(), 2);
  EXPECT_TRUE(e[0].EqualTo(expressions[0]));
  EXPECT_TRUE(e[1].EqualTo(expressions[1]));

  const Vector3d x{.2, .4, .6};
  VectorXd y;
  const Vector2d y_expected{1.04, .76};
  constraint.Eval(x, y);

  EXPECT_TRUE(CompareMatrices(y, y_expected));

  AutoDiffVecXd x_autodiff = drake::math::initializeAutoDiff(x);
  AutoDiffVecXd y_autodiff;
  Eigen::Matrix<double, 2, 3> y_gradient_expected;
  // clang-format off
  y_gradient_expected << .4, 0., 0.,
                         0., .8, 1.;
  // clang-format on
  constraint.Eval(x_autodiff, y_autodiff);

  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(y_autodiff),
                              y_expected));
  EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(y_autodiff),
                              y_gradient_expected));
}

// Test that the Eval() method of LinearComplementarityConstraint correctly
// returns the slack.
GTEST_TEST(testConstraint, testSimpleLCPConstraintEval) {
  Eigen::Matrix2d M = Eigen::Matrix2d::Identity();
  Eigen::Vector2d q(-1, -1);

  LinearComplementarityConstraint c(M, q);
  Eigen::VectorXd w;

  Eigen::Vector2d x1(1, 1);
  c.Eval(x1, w);
  EXPECT_TRUE(
      CompareMatrices(w, Vector2d(0, 0), 1e-4, MatrixCompareType::absolute));
  EXPECT_TRUE(c.CheckSatisfied(x1));

  Eigen::Vector2d x2(1, 2);
  c.Eval(x2, w);
  EXPECT_TRUE(
      CompareMatrices(w, Vector2d(0, 1), 1e-4, MatrixCompareType::absolute));
  EXPECT_FALSE(c.CheckSatisfied(x2));
}

class SimpleEvaluator : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleEvaluator)
  SimpleEvaluator() : EvaluatorBase(2, 3) {
    c_.resize(2, 3);
    c_ <<
        1, 2, 3,
        4, 5, 6;
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    y = c_ * x;
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y = c_ * x;
  }

 private:
  Eigen::MatrixXd c_;
};

GTEST_TEST(testConstraint, testEvaluatorConstraint) {
  const VectorXd lb = VectorXd::Constant(2, -1);
  const VectorXd ub = VectorXd::Constant(2, 1);
  EvaluatorConstraint<> constraint(std::make_shared<SimpleEvaluator>(), lb, ub);
  EXPECT_EQ(3, constraint.num_vars());
  EXPECT_EQ(2, constraint.num_constraints());
  EXPECT_EQ(lb, constraint.lower_bound());
  EXPECT_EQ(ub, constraint.upper_bound());
  VectorXd x(3);
  x << 7, 8, 9;
  VectorXd y(2);
  MatrixXd c(2, 3);
  c << 1, 2, 3,
       4, 5, 6;
  const VectorXd y_expected = c * x;
  constraint.Eval(x, y);
  EXPECT_EQ(y_expected, y);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
