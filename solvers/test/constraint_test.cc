#include "drake/solvers/constraint.h"

#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using ::testing::HasSubstr;
using ::testing::Not;

namespace drake {

using symbolic::test::ExprEqual;
using symbolic::test::FormulaEqual;

namespace solvers {

using symbolic::Environment;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

namespace {
const double kInf = std::numeric_limits<double>::infinity();

// Given a list of variables [x₀, ..., xₙ] and a list of values [v₀, ..., vₙ],
// returns an environment {x₀ ↦ v₀, ..., xₙ ↦ vₙ}.
Environment BuildEnvironment(const VectorX<Variable>& vars,
                             const VectorXd& values) {
  Environment env;
  for (int i = 0; i < vars.size(); ++i) {
    env.insert(vars[i], values[i]);
  }
  return env;
}

GTEST_TEST(TestConstraint, BoundSizeCheck) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      LinearConstraint(Eigen::Matrix3d::Identity(), Eigen::Vector2d(1., 2),
                       Eigen::Vector3d(2., 3, 4.)),
      "Constraint  expects lower and upper bounds of size 3, got lower "
      "bound of size 2 and upper bound of size 3.");
}

GTEST_TEST(TestConstraint, LinearConstraintSparse) {
  // Construct LinearConstraint with sparse A matrix.
  std::vector<Eigen::Triplet<double>> A_triplets;
  A_triplets.emplace_back(0, 1, 0.5);
  A_triplets.emplace_back(1, 0, 1.5);
  Eigen::SparseMatrix<double> A_sparse(2, 3);
  A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::Vector2d lb(0, 1);
  Eigen::Vector2d ub(1, 2);
  LinearConstraint dut(A_sparse, lb, ub);
  EXPECT_EQ(dut.num_vars(), 3);
  EXPECT_EQ(dut.num_constraints(), 2);
  EXPECT_EQ(dut.get_sparse_A().nonZeros(), A_sparse.nonZeros());
  EXPECT_TRUE(
      CompareMatrices(dut.get_sparse_A().toDense(), A_sparse.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.GetDenseA(), A_sparse.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), lb));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), ub));

  // Call UpdateCoefficients with sparse A;
  A_triplets.emplace_back(1, 2, 2.5);
  Eigen::SparseMatrix<double> A_sparse_new(2, 3);
  A_sparse_new.setFromTriplets(A_triplets.begin(), A_triplets.end());
  lb << 1, 4;
  ub << 2, 5;
  dut.UpdateCoefficients(A_sparse_new, lb, ub);
  EXPECT_EQ(dut.get_sparse_A().nonZeros(), A_sparse_new.nonZeros());
  EXPECT_TRUE(
      CompareMatrices(dut.get_sparse_A().toDense(), A_sparse_new.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.GetDenseA(), A_sparse_new.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), lb));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), ub));
}

GTEST_TEST(TestConstraint, LinearEqualityConstraintSparse) {
  std::vector<Eigen::Triplet<double>> A_triplets;
  A_triplets.emplace_back(0, 1, 0.5);
  A_triplets.emplace_back(1, 0, 1.5);
  Eigen::SparseMatrix<double> A_sparse(2, 3);
  A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
  Eigen::Vector2d bound(0, 1);
  LinearEqualityConstraint dut(A_sparse, bound);
  EXPECT_EQ(dut.get_sparse_A().nonZeros(), A_sparse.nonZeros());
  EXPECT_TRUE(
      CompareMatrices(dut.get_sparse_A().toDense(), A_sparse.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.GetDenseA(), A_sparse.toDense()));
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), bound));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), bound));
}

GTEST_TEST(TestConstraint, testLinearConstraintUpdate) {
  // Update the coefficients or the bound of the linear constraint, and check
  // the updated constraint.
  const Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  const Eigen::Vector2d b(1, 2);
  LinearEqualityConstraint constraint(A, b);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), b));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), b));
  EXPECT_TRUE(CompareMatrices(constraint.GetDenseA(), A));
  EXPECT_EQ(constraint.num_constraints(), 2);

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{symbolic::MakeVectorContinuousVariable(2, "x")};
  VectorX<Expression> y_sym;
  constraint.Eval(x_sym, &y_sym);
  EXPECT_EQ(y_sym.size(), 2);
  EXPECT_PRED2(ExprEqual, y_sym[0], x_sym[0]);
  EXPECT_PRED2(ExprEqual, y_sym[1], x_sym[1]);
  EXPECT_PRED2(FormulaEqual, constraint.CheckSatisfied(x_sym),
               1 == x_sym[0] && 2 == x_sym[1]);

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
  EXPECT_TRUE(CompareMatrices(constraint.GetDenseA(), A3));
  EXPECT_TRUE(CompareMatrices(constraint.get_sparse_A().toDense(), A3));
  EXPECT_EQ(constraint.num_constraints(), 3);
}

GTEST_TEST(testConstraint, testRemoveTinyCoefficient) {
  Eigen::Matrix<double, 2, 3> A;
  const double tol = 1E-8;
  // clang-format off
  A << 0.5 * tol, -0.5 * tol, 0,
       1.5, -0.1 * tol, 0;
  // clang-format on
  Eigen::Vector2d lb(-0.1 * tol, 0);
  Eigen::Vector2d ub(2, 0.1 * tol);
  LinearConstraint dut(A, lb, ub);
  dut.RemoveTinyCoefficient(tol);
  Eigen::Matrix<double, 2, 3> A_expected;
  // clang-format off
  A_expected << 0, 0, 0,
                1.5, 0, 0;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(dut.get_sparse_A().toDense(), A_expected));
  EXPECT_TRUE(CompareMatrices(dut.GetDenseA(), A_expected));
  EXPECT_TRUE(CompareMatrices(dut.lower_bound(), lb));
  EXPECT_TRUE(CompareMatrices(dut.upper_bound(), ub));

  DRAKE_EXPECT_THROWS_MESSAGE(dut.RemoveTinyCoefficient(-1),
                              ".*tol should be non-negative");
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
  EXPECT_EQ(constraint1.hessian_type(),
            QuadraticConstraint::HessianType::kPositiveSemidefinite);
  // The constraint is non-convex due to the lower bound not being -inf.
  EXPECT_FALSE(constraint1.is_convex());
  std::ostringstream os;
  constraint1.Display(os, symbolic::MakeVectorContinuousVariable(2, "x"));
  EXPECT_EQ(os.str(),
            "QuadraticConstraint\n"
            "0 <= (x(0) + 2 * x(1) + 0.5 * pow(x(0), 2) + 0.5 * pow(x(1), 2)) "
            "<= 1\n");

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{symbolic::MakeVectorContinuousVariable(2, "x")};
  const Variable& x0{x_sym[0]};
  const Variable& x1{x_sym[1]};
  VectorX<Expression> y_sym;
  constraint1.Eval(x_sym, &y_sym);
  EXPECT_EQ(y_sym.size(), 1);
  EXPECT_PRED2(ExprEqual, y_sym[0],
               0.5 * x0 * x0 + 0.5 * x1 * x1 + x0 + 2 * x1);
  EXPECT_PRED2(FormulaEqual, constraint1.CheckSatisfied(x_sym),
               0 <= y_sym[0] && y_sym[0] <= 1);

  // Updates constraint with a non-symmetric negative definite Hessian.
  // clang-format off
  Q << -1, 1,
       0, -1;
  // clang-format on
  b << 1, 2;
  constraint1.UpdateCoefficients(Q, b);
  EXPECT_TRUE(CompareMatrices(constraint1.Q(), (Q + Q.transpose()) / 2));
  EXPECT_TRUE(CompareMatrices(constraint1.b(), b));
  EXPECT_EQ(constraint1.hessian_type(),
            QuadraticConstraint::HessianType::kNegativeSemidefinite);
  EXPECT_FALSE(constraint1.is_convex());

  // Constructs a constraint with a non-symmetric Hessian.
  QuadraticConstraint constraint2(
      Q, b, 0, kInf, QuadraticConstraint::HessianType::kNegativeSemidefinite);
  EXPECT_TRUE(CompareMatrices(constraint2.Q(), (Q + Q.transpose()) / 2));
  EXPECT_TRUE(CompareMatrices(constraint2.b(), b));
  EXPECT_EQ(constraint2.hessian_type(),
            QuadraticConstraint::HessianType::kNegativeSemidefinite);
  EXPECT_TRUE(constraint2.is_convex());

  // Updates constraints with an indefinite Hessian.
  // clang-format off
  Q << 1, 2,
       2, 3;
  // clang-format on
  constraint2.UpdateCoefficients(Q, b);
  EXPECT_EQ(constraint2.hessian_type(),
            QuadraticConstraint::HessianType::kIndefinite);
  EXPECT_FALSE(constraint2.is_convex());

  // Updates constraint with a specified Hessian type.
  constraint2.UpdateCoefficients(
      Eigen::Matrix2d::Identity(), b,
      QuadraticConstraint::HessianType::kPositiveSemidefinite);
  EXPECT_EQ(constraint2.hessian_type(),
            QuadraticConstraint::HessianType::kPositiveSemidefinite);
  EXPECT_FALSE(constraint2.is_convex());

  // Construct a constraint with psd Hessian and lower bound being -inf.
  QuadraticConstraint constraint3(Eigen::Matrix2d::Identity(), b, -kInf, 1);
  EXPECT_TRUE(constraint3.is_convex());
}

void TestLorentzConeEvalConvex(const Eigen::Ref<const Eigen::MatrixXd>& A,
                               const Eigen::Ref<const Eigen::VectorXd>& b,
                               const VectorXd& x_test) {
  LorentzConeConstraint cnstr1(A, b, LorentzConeConstraint::EvalType::kConvex);
  LorentzConeConstraint cnstr2(A, b,
                               LorentzConeConstraint::EvalType::kConvexSmooth);
  EXPECT_EQ(cnstr1.eval_type(), LorentzConeConstraint::EvalType::kConvex);
  EXPECT_EQ(cnstr2.eval_type(), LorentzConeConstraint::EvalType::kConvexSmooth);
  EXPECT_EQ(cnstr1.num_constraints(), 1);
  EXPECT_EQ(cnstr2.num_constraints(), 1);
  EXPECT_TRUE(CompareMatrices(cnstr1.lower_bound(), Vector1d(0)));
  EXPECT_TRUE(CompareMatrices(cnstr2.lower_bound(), Vector1d(0)));
  EXPECT_TRUE(CompareMatrices(cnstr1.upper_bound(), Vector1d(kInf)));
  EXPECT_TRUE(CompareMatrices(cnstr2.upper_bound(), Vector1d(kInf)));
  VectorXd y1, y2;
  cnstr1.Eval(x_test, &y1);
  cnstr2.Eval(x_test, &y2);
  VectorXd z = A * x_test + b;
  Vector1d y_expected(z(0) - z.tail(z.rows() - 1).norm());
  EXPECT_TRUE(CompareMatrices(y1, y_expected, 1e-12));
  EXPECT_TRUE(CompareMatrices(y2, y_expected, 1e-12));

  std::ostringstream os;
  cnstr1.Display(
      os, symbolic::MakeVectorContinuousVariable(cnstr1.num_vars(), "x"));
  EXPECT_THAT(os.str(), HasSubstr("LorentzConeConstraint\n"));
  EXPECT_THAT(os.str(), HasSubstr("pow"));
  EXPECT_THAT(os.str(), HasSubstr("sqrt"));

  Eigen::MatrixXd dx_test(x_test.rows(), 2);
  dx_test.col(0) = Eigen::VectorXd::LinSpaced(x_test.rows(), 0, 1);
  dx_test.col(1) = Eigen::VectorXd::LinSpaced(x_test.rows(), 1, 2);

  const AutoDiffVecXd x_autodiff = math::InitializeAutoDiff(x_test, dx_test);

  AutoDiffVecXd y_autodiff1, y_autodiff2;
  cnstr1.Eval(x_autodiff, &y_autodiff1);
  cnstr2.Eval(x_autodiff, &y_autodiff2);
  EXPECT_TRUE(
      CompareMatrices(y_expected, math::ExtractValue(y_autodiff1), 1e-12));
  EXPECT_TRUE(
      CompareMatrices(y_expected, math::ExtractValue(y_autodiff2), 1e-12));
  // With eval_type = kConvexSmooth, we approximate the gradient with some
  // smooth function, which introduces larger error (2e-12).
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff1),
                              math::ExtractGradient(y_autodiff2), 2e-12));
}

// Tests if the Lorentz Cone constraint (with non-convex eval) is imposed
// correctly.
void TestLorentzConeEvalNonconvex(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                  const Eigen::Ref<const Eigen::VectorXd>& b,
                                  const VectorXd& x_test, bool is_in_cone) {
  LorentzConeConstraint cnstr(A, b,
                              LorentzConeConstraint::EvalType::kNonconvex);
  EXPECT_EQ(cnstr.num_constraints(), 2);
  EXPECT_TRUE(CompareMatrices(cnstr.lower_bound(), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(cnstr.upper_bound(), Eigen::Vector2d::Constant(kInf)));
  VectorXd y;
  // Test Eval with VectorXd.
  cnstr.Eval(x_test, &y);
  Vector2d y_expected;
  VectorXd z = A * x_test + b;
  y_expected(0) = z(0);
  y_expected(1) = z(0) * z(0) - z.tail(z.size() - 1).squaredNorm();
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected = (y(0) >= 0) && (y(1) >= 0);
  EXPECT_EQ(is_in_cone, is_in_cone_expected);
  EXPECT_EQ(cnstr.CheckSatisfied(x_test), is_in_cone_expected);

  std::ostringstream os;
  cnstr.Display(os,
                symbolic::MakeVectorContinuousVariable(cnstr.num_vars(), "x"));
  EXPECT_THAT(os.str(), HasSubstr("LorentzConeConstraint\n"));
  EXPECT_THAT(os.str(), HasSubstr("pow"));
  EXPECT_THAT(os.str(), Not(HasSubstr("sqrt")));

  auto tx = drake::math::InitializeAutoDiff(x_test);
  AutoDiffVecXd x_taylor = tx;
  AutoDiffVecXd y_taylor;
  // Test Eval with AutoDiff.
  cnstr.Eval(x_taylor, &y_taylor);

  EXPECT_TRUE(CompareMatrices(y, math::ExtractValue(y_taylor)));
  EXPECT_EQ(cnstr.CheckSatisfied(x_taylor), is_in_cone_expected);

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{
      symbolic::MakeVectorContinuousVariable(x_test.size(), "x")};
  VectorX<Expression> y_sym;
  cnstr.Eval(x_sym, &y_sym);
  const Environment env{BuildEnvironment(x_sym, x_test)};
  EXPECT_TRUE(CompareMatrices(Evaluate(y_sym, env), y_expected, 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_EQ(cnstr.CheckSatisfied(x_sym).Evaluate(env), is_in_cone_expected);
}

void TestRotatedLorentzConeEval(const Eigen::Ref<const Eigen::MatrixXd> A,
                                const Eigen::Ref<const Eigen::VectorXd> b,
                                const VectorXd& x_test, bool is_in_cone) {
  RotatedLorentzConeConstraint cnstr(A, b);
  VectorXd y;
  cnstr.Eval(x_test, &y);
  Eigen::VectorXd z = A * x_test + b;
  Vector3d y_expected(z(0), z(1),
                      z(0) * z(1) - z.tail(z.size() - 2).squaredNorm());
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected = (z(0) >= 0) && (z(1) >= 0) &&
                             (z(0) * z(1) >= z.tail(z.size() - 2).norm());
  EXPECT_EQ(is_in_cone, is_in_cone_expected);
  EXPECT_EQ(cnstr.CheckSatisfied(x_test), is_in_cone_expected);

  // Eval with taylor var.
  auto tx = drake::math::InitializeAutoDiff(x_test);
  AutoDiffVecXd x_taylor = tx;
  AutoDiffVecXd y_taylor;
  cnstr.Eval(x_taylor, &y_taylor);

  EXPECT_TRUE(CompareMatrices(y, math::ExtractValue(y_taylor)));
  EXPECT_EQ(cnstr.CheckSatisfied(x_taylor), is_in_cone_expected);

  std::ostringstream os;
  cnstr.Display(os,
                symbolic::MakeVectorContinuousVariable(cnstr.num_vars(), "x"));
  EXPECT_THAT(os.str(), HasSubstr("RotatedLorentzConeConstraint\n"));
  EXPECT_THAT(os.str(), HasSubstr("pow"));

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{
      symbolic::MakeVectorContinuousVariable(x_test.size(), "x")};
  VectorX<Expression> y_sym;
  cnstr.Eval(x_sym, &y_sym);
  const Environment env{BuildEnvironment(x_sym, x_test)};
  EXPECT_TRUE(CompareMatrices(Evaluate(y_sym, env), y_expected, 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_EQ(cnstr.CheckSatisfied(x_sym).Evaluate(env), is_in_cone_expected);
}

GTEST_TEST(testConstraint, testLorentzConeConstraint) {
  // [3;1;1] is in the interior of the Lorentz cone.
  Eigen::Vector3d x1(3.0, 1.0, 1.0);
  TestLorentzConeEvalConvex(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(), x1);
  TestLorentzConeEvalNonconvex(Eigen::Matrix3d::Identity(),
                               Eigen::Vector3d::Zero(), x1, true);

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
  TestLorentzConeEvalConvex(A2, b2, x2);
  TestLorentzConeEvalNonconvex(A2, b2, x2, true);

  // [3; 3; 1] is outside of the Lorentz cone.
  Eigen::Vector4d x3(1, -1, 2, 3);
  Eigen::Matrix<double, 3, 4> A3;
  // clang-format off
  A3 << 1, 0, -1, 2,
        -1, 2, 0, 1,
        0, -2, 3, 1;
  // clang-format on
  Eigen::Vector3d b3 = Eigen::Vector3d(3, 3, 1) - A3 * x3;
  TestLorentzConeEvalConvex(A3, b3, x3);
  TestLorentzConeEvalNonconvex(A3, b3, x3, false);

  // [-3; 1; 1] is outside of the Lorentz cone.
  Vector1d x4 = Vector1d::Constant(4);
  Eigen::Vector3d A4(-1, 3, 2);
  Eigen::Vector3d b4 = Eigen::Vector3d(-3, 1, 1) - A4 * x4;
  TestLorentzConeEvalConvex(A4, b4, x4);
  TestLorentzConeEvalNonconvex(A4, b4, x4, false);
}

GTEST_TEST(testConstraint, testLorentzConeConstraintAtZeroZ) {
  // Test LorentzConeConstraint with smoothed approximated gradient  evaluated
  // at z = 0
  Vector2d x(1, 2);
  Eigen::Matrix<double, 3, 2> A;
  A << 1, 2, -2, -1, 2, 3;
  Eigen::Vector3d b = -A * x;
  LorentzConeConstraint cnstr(A, b,
                              LorentzConeConstraint::EvalType::kConvexSmooth);
  AutoDiffVecXd y_autodiff;
  cnstr.Eval(math::InitializeAutoDiff(x), &y_autodiff);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff), Vector1d(0)));
  const Eigen::MatrixXd y_gradient = math::ExtractGradient(y_autodiff);
  // The gradient of dy/dz is [1, 0, 0], so the dy/dx = dy/dz * dz/dx = dy/dz *
  // A = A.row(0).
  EXPECT_TRUE(CompareMatrices(y_gradient, A.row(0)));
}

GTEST_TEST(testConstraint, LorentzConeConstraintUpdateCoefficients) {
  Eigen::Matrix<double, 3, 2> A;
  A << 1, 2, -2, -1, 2, 3;
  Eigen::Vector3d b(1, 2, 3);
  LorentzConeConstraint constraint(
      A, b, LorentzConeConstraint::EvalType::kConvexSmooth);
  const int num_constraints = constraint.num_constraints();
  A *= 2;
  b *= 3;
  constraint.UpdateCoefficients(A, b);
  EXPECT_TRUE(CompareMatrices(constraint.A().toDense(), A));
  EXPECT_TRUE(CompareMatrices(constraint.A_dense(), A));
  EXPECT_TRUE(CompareMatrices(constraint.b(), b));

  // Now try A with different number of rows. UpdateCoefficients should still
  // work.
  Eigen::Matrix<double, 4, 2> new_A;
  new_A << Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity();
  Eigen::Vector4d new_b = Eigen::Vector4d::Zero();
  constraint.UpdateCoefficients(new_A, new_b);
  EXPECT_EQ(constraint.num_vars(), 2);
  EXPECT_EQ(constraint.num_constraints(), num_constraints);

  DRAKE_EXPECT_THROWS_MESSAGE(
      constraint.UpdateCoefficients(Eigen::Matrix3d::Identity(),
                                    Eigen::Vector3d::Zero()),
      ".*UpdateCoefficients uses new_A with 3 columns to update a constraint "
      "with 2 variables.");
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

GTEST_TEST(testConstraint, RotatedLorentzConeConstraintUpdateCoefficients) {
  Eigen::Matrix<double, 3, 2> A;
  A << 1, 2, -2, -1, 2, 3;
  Eigen::Vector3d b(1, 2, 3);
  RotatedLorentzConeConstraint constraint(A, b);
  const int num_constraints = constraint.num_constraints();
  A *= 2;
  b *= 3;
  constraint.UpdateCoefficients(A, b);
  EXPECT_TRUE(CompareMatrices(constraint.A().toDense(), A));
  EXPECT_TRUE(CompareMatrices(constraint.A_dense(), A));
  EXPECT_TRUE(CompareMatrices(constraint.b(), b));
  EXPECT_EQ(constraint.num_vars(), 2);

  // Now try A with different number of rows. UpdateCoefficients should still
  // work.
  Eigen::Matrix<double, 4, 2> new_A;
  new_A << Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity();
  Eigen::Vector4d new_b = Eigen::Vector4d::Zero();
  constraint.UpdateCoefficients(new_A, new_b);
  EXPECT_EQ(constraint.num_vars(), 2);
  EXPECT_EQ(constraint.num_constraints(), num_constraints);

  DRAKE_EXPECT_THROWS_MESSAGE(
      constraint.UpdateCoefficients(Eigen::Matrix3d::Identity(),
                                    Eigen::Vector3d::Zero()),
      ".*UpdateCoefficients uses new_A with 3 columns to update a constraint "
      "with 2 variables.");
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
  cnstr.Eval(X1, &y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());
  EXPECT_TRUE(cnstr.CheckSatisfied(X1));

  Eigen::Matrix<double, 9, 1> X2;
  // clang-format off
  X2 << 1, 2, 0,
        2, -2, -1,
        0, -1, -2;
  // clang-format on
  cnstr.Eval(X2, &y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
  EXPECT_EQ(cnstr.matrix_rows(), 3);
  EXPECT_FALSE(cnstr.CheckSatisfied(X2));

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{
      symbolic::MakeVectorContinuousVariable(X1.size(), "x")};
  VectorX<Expression> y_sym;
  EXPECT_THROW(cnstr.Eval(x_sym, &y_sym), std::logic_error);
  EXPECT_THROW(cnstr.CheckSatisfied(x_sym), std::logic_error);
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
  cnstr.Eval(x1, &y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());
  EXPECT_TRUE(cnstr.CheckSatisfied(x1));

  // [1 -2]
  // [-2 1] is not p.s.d
  Eigen::Vector2d x2(0, -1);
  cnstr.Eval(x2, &y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
  EXPECT_FALSE(cnstr.CheckSatisfied(x2));

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{
      symbolic::MakeVectorContinuousVariable(x1.size(), "x")};
  VectorX<Expression> y_sym;
  EXPECT_THROW(cnstr.Eval(x_sym, &y_sym), std::logic_error);
  EXPECT_THROW(cnstr.CheckSatisfied(x_sym), std::logic_error);
}

GTEST_TEST(testConstraint, testExpressionConstraint) {
  Variable x0{"x0"};
  Variable x1{"x1"};
  Variable x2{"x2"};

  Vector3<Variable> vars{x0, x1, x2};
  Vector2<Expression> e{1. + x0 * x0, x1 * x1 + x2};

  ExpressionConstraint constraint(e, Vector2d::Zero(), 2. * Vector2d::Ones());

  const VectorX<symbolic::Expression>& expressions{constraint.expressions()};
  ASSERT_EQ(expressions.size(), 2);
  EXPECT_TRUE(e[0].EqualTo(expressions[0]));
  EXPECT_TRUE(e[1].EqualTo(expressions[1]));

  std::ostringstream os;
  constraint.Display(os, vars);
  EXPECT_EQ(os.str(),
            "ExpressionConstraint\n"
            "0 <= (1 + pow(x0, 2)) <= 2\n"
            "0 <= (x2 + pow(x1, 2)) <= 2\n");

  const Vector3d x{.2, .4, .6};
  VectorXd y;
  const Vector2d y_expected{1.04, .76};
  constraint.Eval(x, &y);

  EXPECT_TRUE(CompareMatrices(y, y_expected));

  AutoDiffVecXd x_autodiff = drake::math::InitializeAutoDiff(x);
  AutoDiffVecXd y_autodiff;
  Eigen::Matrix<double, 2, 3> y_gradient_expected;
  // clang-format off
  y_gradient_expected << .4, 0., 0.,
                         0., .8, 1.;
  // clang-format on
  constraint.Eval(x_autodiff, &y_autodiff);

  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff), y_expected));
  EXPECT_TRUE(
      CompareMatrices(math::ExtractGradient(y_autodiff), y_gradient_expected));

  // Test Eval/CheckSatisfied using Expression.
  VectorX<Expression> y_sym;
  constraint.Eval(vars, &y_sym);
  EXPECT_EQ(y_sym.size(), e.size());
  EXPECT_PRED2(ExprEqual, y_sym[0], e[0]);
  EXPECT_PRED2(ExprEqual, y_sym[1], e[1]);
  EXPECT_PRED2(FormulaEqual, constraint.CheckSatisfied(vars),
               0 <= e[0] && e[0] <= 2 && 0 <= e[1] && e[1] <= 2);
}

// Test that the Eval() method of LinearComplementarityConstraint correctly
// returns the slack.
GTEST_TEST(testConstraint, testSimpleLCPConstraintEval) {
  Eigen::Matrix2d M = Eigen::Matrix2d::Identity();
  Eigen::Vector2d q(-1, -1);

  LinearComplementarityConstraint c(M, q);
  Eigen::VectorXd w;

  Eigen::Vector2d x1(1, 1);
  c.Eval(x1, &w);
  EXPECT_TRUE(
      CompareMatrices(w, Vector2d(0, 0), 1e-4, MatrixCompareType::absolute));
  EXPECT_TRUE(c.CheckSatisfied(x1));

  Eigen::Vector2d x2(1, 2);
  c.Eval(x2, &w);
  EXPECT_TRUE(
      CompareMatrices(w, Vector2d(0, 1), 1e-4, MatrixCompareType::absolute));
  EXPECT_FALSE(c.CheckSatisfied(x2));

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{
      symbolic::MakeVectorContinuousVariable(x1.size(), "x")};
  const Variable& x_0{x_sym[0]};
  const Variable& x_1{x_sym[1]};
  VectorX<Expression> y_sym;
  c.Eval(x_sym, &y_sym);  // y = Mx + q = Ix + [-1, -1].
  EXPECT_EQ(y_sym.size(), 2);
  EXPECT_PRED2(ExprEqual, y_sym[0], x_0 - 1);
  EXPECT_PRED2(ExprEqual, y_sym[1], x_1 - 1);
  // 1. Mx + q = Ix + [-1 -1] >= 0
  // 2. x >= 0
  // 3. x'(Mx + q) = x₀(x₀ - 1) + x₁(x₁ - 1) == 0
  EXPECT_PRED2(FormulaEqual, c.CheckSatisfied(x_sym),
               x_0 - 1.0 >= 0 && x_1 - 1.0 >= 0 && x_0 >= 0.0 && x_1 >= 0.0 &&
                   x_0 * (x_0 - 1.0) + x_1 * (x_1 - 1.0) == 0.0);
}

class SimpleEvaluator : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleEvaluator)
  SimpleEvaluator() : EvaluatorBase(2, 3) {
    c_.resize(2, 3);
    // clang-format off
    c_ << 1, 2, 3,
          4, 5, 6;
    // clang-format on
  }

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<Variable>>& x,
              VectorX<Expression>* y) const override {
    DoEvalGeneric(x, y);
  }

 private:
  template <typename DerivedX, typename ScalarY>
  void DoEvalGeneric(const Eigen::MatrixBase<DerivedX>& x,
                     VectorX<ScalarY>* y) const {
    *y = c_ * x.template cast<ScalarY>();
  }

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
  // clang-format off
  c << 1, 2, 3,
       4, 5, 6;
  // clang-format on
  const VectorXd y_expected = c * x;
  constraint.Eval(x, &y);
  EXPECT_EQ(y_expected, y);

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{symbolic::MakeVectorContinuousVariable(3, "x")};
  const Variable& x_0{x_sym[0]};
  const Variable& x_1{x_sym[1]};
  const Variable& x_2{x_sym[2]};

  VectorX<Expression> y_sym;
  constraint.Eval(x_sym, &y_sym);  // y = c * x
  EXPECT_EQ(y_sym.size(), 2);
  EXPECT_PRED2(ExprEqual, y_sym[0], 1 * x_0 + 2 * x_1 + 3 * x_2);
  EXPECT_PRED2(ExprEqual, y_sym[1], 4 * x_0 + 5 * x_1 + 6 * x_2);
  EXPECT_PRED2(
      FormulaEqual, constraint.CheckSatisfied(x_sym),
      -1 <= y_sym[0] && y_sym[0] <= 1 && -1 <= y_sym[1] && y_sym[1] <= 1);
}

GTEST_TEST(testConstraint, testExponentialConeConstraint) {
  Eigen::SparseMatrix<double> A(3, 2);
  A.coeffRef(0, 0) = 2;
  A.coeffRef(1, 1) = 3;
  A.coeffRef(2, 0) = 1;
  Eigen::Vector3d b(1, 2, 3);
  ExponentialConeConstraint constraint(A, b);

  Eigen::Vector2d x(3, 4);
  Eigen::VectorXd y;
  constraint.Eval(x, &y);
  // Now evaluate z manually.
  const Eigen::Vector3d z = A * x + b;
  Eigen::Vector2d y_expected;
  y_expected(0) = z(0) - z(1) * std::exp(z(2) / z(1));
  y_expected(1) = z(1);
  const double tol = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  // Check autodiff evaluation.
  Eigen::MatrixXd dx(2, 1);
  dx << 1, 1;
  const auto x_autodiff = math::InitializeAutoDiff(Eigen::VectorXd(x), dx);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_autodiff, &y_autodiff);
  // Now compute the gradient manually.
  const AutoDiffVecXd z_autodiff = A * x_autodiff + b;
  AutoDiffVecXd y_autodiff_expected(2);
  y_autodiff_expected(0) =
      z_autodiff(0) - z_autodiff(1) * exp(z_autodiff(2) / z_autodiff(1));
  y_autodiff_expected(1) = z_autodiff(1);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff), y_expected, tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff),
                              math::ExtractGradient(y_autodiff_expected), tol));
}

}  // namespace
}  // namespace solvers
}  // namespace drake
