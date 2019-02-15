#include "drake/solvers/non_convex_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/solve.h"

namespace drake {
using symbolic::test::ExprEqual;
using symbolic::test::PolynomialEqual;
using symbolic::test::FormulaEqual;
namespace solvers {
namespace {
GTEST_TEST(DecomposeNonConvexQuadraticForm, Test0) {
  // Decomposes a PSD matrix Q. This should yield Q1 = Q and Q2 = 0.
  Eigen::Matrix3d Q1, Q2;
  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(CompareMatrices(Q1, Q, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Q2, Eigen::Matrix3d::Zero(), 1E-5,
                              MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test1) {
  // Decomposes a negative definite matrix Q. This should yield Q1 = 0, and Q2 =
  // -Q.
  Eigen::Matrix3d Q1, Q2;
  Eigen::Matrix3d Q = -Eigen::Matrix3d::Identity();
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(CompareMatrices(Q1, Eigen::Matrix3d::Zero(), 1E-5,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Q2, -Q, 1E-5, MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test2) {
  // Decomposes an indefinite matrix Q, [0, 1; 1 0].
  // This should yield Q1 = [1 1; 1 1]/2, Q2 = [1 -1; -1 1]/2.
  Eigen::Matrix2d Q1, Q2;
  Eigen::Matrix2d Q;
  Q << 0, 1, 1, 0;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  Eigen::Matrix2d Q1_expected, Q2_expected;
  Q1_expected << 1.0 / 2, 1.0 / 2, 1.0 / 2, 1.0 / 2;
  Q2_expected << 1.0 / 2, -1.0 / 2, -1.0 / 2, 1.0 / 2;
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));

  // Decomposes another indefinite matrix Q, [0, 2; 0, 0], this matrix has the
  // same quadratic form as [0, 1; 1 0], so it should give the same Q1 and Q2.
  Q << 0, 2, 0, 0;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test3) {
  // Decomposes an indefinite matrix Q = [1 3; 1, 1].
  // This should yield Q1 = [1.5 1.5; 1.5 1.5], Q2 = [0.5 -0.5; -0.5 0.5]
  Eigen::Matrix2d Q1, Q2;
  Eigen::Matrix2d Q;
  Q << 1, 3, 1, 1;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  Eigen::Matrix2d Q1_expected, Q2_expected;
  Q1_expected << 1.5, 1.5, 1.5, 1.5;
  Q2_expected << 0.5, -0.5, -0.5, 0.5;
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));
}

void CheckRelaxNonConvexQuadraticConstraintInTrustRegion(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p,
    const Eigen::Ref<const VectorXDecisionVariable>& y, double lower_bound,
    double upper_bound,
    const Eigen::Ref<const Eigen::VectorXd>& linearization_point,
    double trust_region_gap) {
  const auto& x0 = linearization_point;
  const auto result = AddRelaxNonConvexQuadraticConstraintInTrustRegion(
      prog, x, Q1, Q2, y, p, lower_bound, upper_bound, linearization_point,
      trust_region_gap);
  Binding<LinearConstraint> linear_constraint = std::get<0>(result);
  EXPECT_EQ(std::get<1>(result).size(), 2);
  Binding<RotatedLorentzConeConstraint> rotated_lorentz_cone_constraint1 =
      std::get<1>(result)[0];
  Binding<RotatedLorentzConeConstraint> rotated_lorentz_cone_constraint2 =
      std::get<1>(result)[1];
  VectorDecisionVariable<2> z = std::get<2>(result);
  Vector3<symbolic::Expression> linear_constraint_expr =
      linear_constraint.evaluator()->A() * linear_constraint.variables();
  Vector3<symbolic::Expression> linear_constraint_expr_expected;
  linear_constraint_expr_expected << z(0) - z(1) + p.dot(y),
      z(0) - 2 * x0.dot(Q1 * x), z(1) - 2 * x0.dot(Q2 * x);
  const double poly_tol{1E-10};
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(PolynomialEqual(
                    symbolic::Polynomial(linear_constraint_expr(i)),
                    symbolic::Polynomial(linear_constraint_expr_expected(i)),
                    poly_tol) ||
                PolynomialEqual(
                    symbolic::Polynomial(linear_constraint_expr(i)),
                    symbolic::Polynomial(-linear_constraint_expr_expected(i)),
                    poly_tol));
  }
  const VectorX<symbolic::Expression> y_lorentz1 =
      rotated_lorentz_cone_constraint1.evaluator()->A() *
          rotated_lorentz_cone_constraint1.variables() +
      rotated_lorentz_cone_constraint1.evaluator()->b();
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(y_lorentz1(0) * y_lorentz1(1)),
                      symbolic::Polynomial(z(0)), poly_tol));
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(
                          y_lorentz1.tail(y_lorentz1.rows() - 2).squaredNorm()),
                      symbolic::Polynomial(x.dot(Q1 * x)), poly_tol));

  const VectorX<symbolic::Expression> y_lorentz2 =
      rotated_lorentz_cone_constraint2.evaluator()->A() *
          rotated_lorentz_cone_constraint2.variables() +
      rotated_lorentz_cone_constraint2.evaluator()->b();
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(y_lorentz2(0) * y_lorentz2(1)),
                      symbolic::Polynomial(z(1)), poly_tol));
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(
                          y_lorentz2.tail(y_lorentz2.rows() - 2).squaredNorm()),
                      symbolic::Polynomial(x.dot(Q2 * x)), poly_tol));
}

Eigen::Matrix<double, 2, 10> GenerateCostDirection() {
  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
      0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  return c;
}

class TestRelaxNonConvexQuadraticConstraintInTrustRegion
    : public ::testing::Test {
 public:
  TestRelaxNonConvexQuadraticConstraintInTrustRegion()
      : prog_{}, x_{prog_.NewContinuousVariables<2>()} {}

 protected:
  MathematicalProgram prog_;
  VectorDecisionVariable<2> x_;
};

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, Test0) {
  Eigen::Matrix2d Q1 = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d Q2;
  Q2 << 1, 0.2, 0.2, 1;
  Eigen::Vector2d p(1, 2);
  const double lower_bound{0};
  const double upper_bound{0.1};
  const Eigen::Vector2d linearization_point(1, 2);
  const double trust_region_gap{1};
  CheckRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Q2, p, x_, lower_bound, upper_bound, linearization_point,
      trust_region_gap);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, Test1) {
  Eigen::Matrix2d Q1;
  Q1 << 1, 0.2, 0.2, 2;
  Eigen::Matrix2d Q2;
  Q2 << 1, 0.3, 0.3, 4;
  Eigen::Vector2d p(3, 2);
  const double lower_bound{-1};
  const double upper_bound{-0.1};
  const Eigen::Vector2d linearization_point(-1, 2);
  const double trust_region_gap{1};
  CheckRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Q2, p, x_, lower_bound, upper_bound, linearization_point,
      trust_region_gap);

  // Check the case in which the variable y in the linear term is not the same
  // as the variable x in the quadratic term.
  auto x_prime = prog_.NewContinuousVariables<2>();
  const Eigen::Vector3d p_prime(3, 2, 1);
  VectorDecisionVariable<3> y{x_(0), x_prime(0), x_prime(1)};
  CheckRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Q2, p_prime, y, lower_bound, upper_bound,
      linearization_point, trust_region_gap);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, TestRuntimeError) {
  Eigen::Matrix2d non_positive_Q;
  non_positive_Q << 1, 1.5, 1.5, 1;
  Eigen::Matrix2d positive_Q;
  positive_Q << 1, 0.2, 0.2, 1;
  // Q1 not being positive semidefinite.
  EXPECT_THROW(AddRelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog_, x_, non_positive_Q, positive_Q, x_,
                   Eigen::Vector2d(1, 0), -1, 0.1, Eigen::Vector2d(1, 2), 1),
               std::runtime_error);
  // Q2 not being positive semidefinite.
  EXPECT_THROW(AddRelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog_, x_, positive_Q, non_positive_Q, x_,
                   Eigen::Vector2d(1, 0), -1, 0.1, Eigen::Vector2d(1, 2), 1),
               std::runtime_error);
  // Negative trust region gap.
  EXPECT_THROW(AddRelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog_, x_, positive_Q, positive_Q, x_,
                   Eigen::Vector2d(1, 0), -1, 0.1, Eigen::Vector2d(1, 2), -0.1),
               std::runtime_error);
}

void SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::VectorXd>& p, double lower_bound,
    double upper_bound, const Eigen::Ref<const Eigen::VectorXd>& x0,
    double trust_region_gap, const Eigen::Ref<const Eigen::MatrixXd>& c) {
  const auto relaxed_constraints =
      AddRelaxNonConvexQuadraticConstraintInTrustRegion(
          prog, x, Q1, Q2, y, p, lower_bound, upper_bound, x0,
          trust_region_gap);
  VectorDecisionVariable<2> z = std::get<2>(relaxed_constraints);

  auto cost = prog->AddLinearCost(x.cast<symbolic::Expression>().sum());
  for (int i = 0; i < c.cols(); ++i) {
    cost.evaluator()->UpdateCoefficients(c.col(i).transpose());
    auto result = Solve(*prog);
    EXPECT_TRUE(result.is_success());
    auto x_sol = result.GetSolution(x);
    auto y_sol = result.GetSolution(y);
    auto z_sol = result.GetSolution(z);
    const double check_tol{1E-5};
    EXPECT_GE(z_sol(0) - z_sol(1) + p.dot(y_sol), lower_bound - check_tol);
    EXPECT_LE(z_sol(0) - z_sol(1) + p.dot(y_sol), upper_bound + check_tol);
    EXPECT_LE(z_sol(0), 2 * x0.dot(Q1 * x_sol) - x0.dot(Q1 * x0) +
                            trust_region_gap + check_tol);
    EXPECT_LE(z_sol(1), 2 * x0.dot(Q2 * x_sol) - x0.dot(Q2 * x0) +
                            trust_region_gap + check_tol);
    EXPECT_GE(z_sol(0), x_sol.dot(Q1 * x_sol) - check_tol);
    EXPECT_GE(z_sol(1), x_sol.dot(Q2 * x_sol) - check_tol);
    const double original_constraint_value{x_sol.dot((Q1 - Q2) * x_sol) +
                                           p.dot(y_sol)};
    EXPECT_LE(original_constraint_value,
              upper_bound + trust_region_gap + check_tol);
    EXPECT_GE(original_constraint_value,
              lower_bound - trust_region_gap - check_tol);
  }
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, SolveProblem0) {
  // For a problem
  // max c'x
  // s.t 1 <= x(0)² - x(1)² <= 2
  // The relaxation of the constraint is
  // 1 <= z(0) - z(1) <= 2
  // z(0) <= 2x₀(0)x(0) - x₀(0)² + d
  // z(1) <= 2x₀(1)x(1) - x₀(1)² + d
  // z(0) >= x(0)²
  // z(1) >= x(1)²
  Eigen::Matrix2d Q1, Q2;
  Q1 << 1, 0, 0, 0;
  Q2 << 0, 0, 0, 1;
  // We linearize it about x = (2, 1). If we set the violation d to 0.5, within
  // the trust region, there should be a solution.
  const double trust_region_gap{0.5};
  const double lb{1};
  const double ub{2};
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Q2, x_, Eigen::Vector2d::Zero(), lb, ub,
      Eigen::Vector2d(2, 1), trust_region_gap, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, SolveProblem1) {
  Eigen::Matrix2d Q1, Q2;
  Q1 << 1, 0, 0, 2;
  Q2 << 2, 0.1, 0.1, 1;
  const double trust_region_gap{0.5};
  const double lb{1};
  const double ub{2};
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Q2, x_, Eigen::Vector2d(1, 3), lb, ub,
      Eigen::Vector2d(2, 1), trust_region_gap, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, SolveProblem2) {
  // The variable y in the linear term is not the same as the variable x in the
  // quadratic term.
  Eigen::Matrix2d Q1, Q2;
  Q1 << 1, 0, 0, 2;
  Q2 << 2, 0.1, 0.1, 1;
  const double trust_region_gap{0.5};
  const double lb{1};
  const double ub{2};
  const auto c = GenerateCostDirection();
  auto x_prime = prog_.NewContinuousVariables<2>();
  VectorDecisionVariable<3> y{x_(0), x_prime(0), x_prime(1)};
  prog_.AddBoundingBoxConstraint(-1, 1, x_prime);
  SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Q2, y, Eigen::Vector3d(1, 2, 3), lb, ub,
      Eigen::Vector2d(2, 1), trust_region_gap, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion,
       SolveInfeasibleProblem0) {
  // The non-convex problem is infeasible. With small relaxation gap, the
  // relaxed problem should also be infeasible.
  // x(0) = 0
  // x(0)² - x(1)² = 1
  prog_.AddBoundingBoxConstraint(0, 0, x_(0));
  const Eigen::Matrix2d Q1 = Eigen::Vector2d(1, 0).asDiagonal();
  const Eigen::Matrix2d Q2 = Eigen::Vector2d(0, 1).asDiagonal();
  AddRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Q2, x_, Eigen::Vector2d::Zero(), 1, 1,
      Eigen::Vector2d(1, 0), 0.1);

  auto result = Solve(prog_).get_solution_result();
  EXPECT_TRUE(result == SolutionResult::kInfeasible_Or_Unbounded ||
              result == SolutionResult::kInfeasibleConstraints);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegionInfeasible,
           SolveInfeasibleProblem1) {
  // The non-convex problem is feasible, but the solution is far away from the
  // linearization point, so the relaxed problem is infeasible within the trust
  // region.
  // x(0)² - 2 * x(1)² = 1
  // x(0) >= x(1) + 2
  // There is not a solution near x = (-5, 3)
  MathematicalProgram prog1;
  auto x1 = prog1.NewContinuousVariables<2>();
  prog1.AddLinearConstraint(x1(0) >= x1(1) + 2);
  Eigen::Matrix2d Q1 = Eigen::Vector2d(1, 0).asDiagonal();
  Eigen::Matrix2d Q2 = Eigen::Vector2d(0, 2).asDiagonal();
  const auto relaxed_constraint =
      AddRelaxNonConvexQuadraticConstraintInTrustRegion(
          &prog1, x1, Q1, Q2, x1, Eigen::Vector2d::Zero(), 1, 1,
          Eigen::Vector2d(-5, -3), 0.1);

  auto result = Solve(prog1).get_solution_result();
  EXPECT_TRUE(result == SolutionResult::kInfeasibleConstraints ||
              result == SolutionResult::kInfeasible_Or_Unbounded);

  // If we linearize the problem at about (7, -5), then the relaxed problem has
  // a solution around the linearization point.
  MathematicalProgram prog2;
  auto x2 = prog2.NewContinuousVariables<2>();
  prog2.AddLinearConstraint(x2(0) >= x2(1) + 2);
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog2, x2, Q1, Q2, x2, Eigen::Vector2d::Zero(), 1, 1,
      Eigen::Vector2d(7, -5), 1.5, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Q2) {
  // Should throw a runtime error.
  // Both Q1 and Q2 are 0
  EXPECT_THROW(AddRelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog_, x_, Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Zero(),
                   x_, Eigen::Vector2d(1, 2), 1, 2, Eigen::Vector2d(2, 1), 1),
               std::runtime_error);
  // Q1 is zero and upper_bound is infinity.
  EXPECT_THROW(
      AddRelaxNonConvexQuadraticConstraintInTrustRegion(
          &prog_, x_, Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Identity(), x_,
          Eigen::Vector2d(1, 2), 1, std::numeric_limits<double>::infinity(),
          Eigen::Vector2d(2, 1), 1),
      std::runtime_error);
  // Q2 is zero and lower_bound is -infinity.
  EXPECT_THROW(
      AddRelaxNonConvexQuadraticConstraintInTrustRegion(
          &prog_, x_, Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Zero(), x_,
          Eigen::Vector2d(1, 2), -std::numeric_limits<double>::infinity(), 1,
          Eigen::Vector2d(2, 1), 1),
      std::runtime_error);
  // lower_bound is larger than upper_bound
  EXPECT_THROW(AddRelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog_, x_, Eigen::Matrix2d::Identity(),
                   0.1 * Eigen::Matrix2d::Identity(), x_, Eigen::Vector2d(1, 2),
                   2, 1, Eigen::Vector2d(2, 1), 1),
               std::runtime_error);
}

void SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1orQ2(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::VectorXd>& p, double lb, double ub,
    const Eigen::Ref<const Eigen::VectorXd>& x0, double trust_region_gap,
    bool Q1_is_zero, const Eigen::Ref<const Eigen::MatrixXd>& c) {
  Eigen::MatrixXd Q1, Q2;
  if (Q1_is_zero) {
    Q1 = Eigen::MatrixXd::Zero(x.rows(), x.rows());
    Q2 = Q;
  } else {
    Q1 = Q;
    Q2 = Eigen::MatrixXd::Zero(x.rows(), x.rows());
  }
  const auto relaxed_constraints =
      AddRelaxNonConvexQuadraticConstraintInTrustRegion(
          prog, x, Q1, Q2, y, p, lb, ub, x0, trust_region_gap);
  const Binding<LinearConstraint> linear_constraint =
      std::get<0>(relaxed_constraints);
  EXPECT_EQ(std::get<1>(relaxed_constraints).size(), 1);
  const Binding<RotatedLorentzConeConstraint> lorentz_cone1 =
      std::get<1>(relaxed_constraints)[0];
  const VectorDecisionVariable<1> z = std::get<2>(relaxed_constraints);
  Vector2<symbolic::Expression> linear_expr, linear_expr_expected;
  linear_expr =
      linear_constraint.evaluator()->A() * linear_constraint.variables();
  Eigen::Vector2d linear_lb_expected, linear_ub_expected;
  linear_lb_expected << x0.dot(Q * x0) - trust_region_gap, lb;
  linear_ub_expected << std::numeric_limits<double>::infinity(), ub;
  if (Q1_is_zero) {
    linear_expr_expected << 2 * x0.dot(Q2 * x) - z(0), p.dot(y) - z(0);
  } else {
    linear_expr_expected << 2 * x0.dot(Q1 * x) - z(0), p.dot(y) + z(0);
  }
  EXPECT_TRUE(CompareMatrices(linear_constraint.evaluator()->lower_bound(),
                              linear_lb_expected, 1E-15,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_constraint.evaluator()->upper_bound(),
                              linear_ub_expected, 1E-15,
                              MatrixCompareType::absolute));
  const double poly_tol{1E-10};
  for (int i = 0; i < 2; ++i) {
    EXPECT_TRUE(PolynomialEqual(symbolic::Polynomial(linear_expr(i)),
                                symbolic::Polynomial(linear_expr_expected(i)),
                                poly_tol));
  }
  VectorX<symbolic::Expression> y1 =
      lorentz_cone1.evaluator()->A() * lorentz_cone1.variables() +
      lorentz_cone1.evaluator()->b();
  EXPECT_TRUE(PolynomialEqual(symbolic::Polynomial(y1(0) * y1(1)),
                              symbolic::Polynomial(z(0)), poly_tol));
  EXPECT_TRUE(PolynomialEqual(
      symbolic::Polynomial(y1.tail(y1.rows() - 2).squaredNorm()),
      symbolic::Polynomial(x.dot(Q * x)), poly_tol));

  auto cost = prog->AddLinearCost(x.cast<symbolic::Expression>().sum());
  const double z_sign = Q1_is_zero ? -1 : 1;
  const double Q_sign = Q1_is_zero ? -1 : 1;
  for (int i = 0; i < c.cols(); ++i) {
    cost.evaluator()->UpdateCoefficients(c.col(i).transpose());
    auto result = Solve(*prog);
    EXPECT_TRUE(result.is_success());
    const double z_sol = result.GetSolution(z(0));
    const Eigen::Vector2d x_sol = result.GetSolution(x);
    const Eigen::VectorXd y_sol = result.GetSolution(y);
    const double tol{1E-5};
    EXPECT_GE(z_sign * z_sol + p.dot(y_sol), lb - tol);
    EXPECT_LE(z_sign * z_sol + p.dot(y_sol), ub + tol);
    EXPECT_LE(z_sol,
              2 * x0.dot(Q * x_sol) - x0.dot(Q * x0) + trust_region_gap + tol);
    EXPECT_GE(z_sol, x_sol.dot(Q * x_sol) - tol);
    const double original_constraint_val{x_sol.dot(Q_sign * Q * x_sol) +
                                         p.dot(y_sol)};
    EXPECT_GE(original_constraint_val, lb - trust_region_gap - tol);
    EXPECT_LE(original_constraint_val, ub + trust_region_gap + tol);
  }
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Test0) {
  // -x(0)² - x(1)² - x(0)x(1) + 2*x(0) <= 1
  Eigen::Matrix2d Q;
  Q << 1, 0.5, 0.5, 1;
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1orQ2(
      &prog_, x_, Q, x_, Eigen::Vector2d(2, 0),
      -std::numeric_limits<double>::infinity(), 1, Eigen::Vector2d(1, 2), 1,
      true, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Test1) {
  // -1 <= -x(0)² - 4x(1)² - 2x(0)x(1) + 3x(0) + 2x(1) <= 4
  Eigen::Matrix2d Q;
  Q << 1, 1, 1, 4;
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1orQ2(
      &prog_, x_, Q, x_, Eigen::Vector2d(3, 2), -1, 4, Eigen::Vector2d(1, 0), 1,
      true, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Test2) {
  // The original non-convex problem is infeasible.
  // -4 <= -x(0)² - x(1)² <= -1
  // x(0) + x(1) >= 10
  Eigen::Matrix2d Q2;
  Q2 << 1, 0, 0, 1;
  prog_.AddLinearConstraint(x_(0) + x_(1) >= 10);

  AddRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Eigen::Matrix2d::Zero(), Q2, x_, Eigen::Vector2d::Zero(), -4,
      -1, Eigen::Vector2d(0, 1), 1);

  auto result = Solve(prog_).get_solution_result();
  EXPECT_TRUE(result == SolutionResult::kInfeasible_Or_Unbounded ||
              result == SolutionResult::kInfeasibleConstraints);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Test3) {
  // The variable y in the linear term is not the same as the variable x in the
  // quadratic term.
  // -1 <= -x(0)² - 4x(1)² - 2x(0)x(1) + 2x(1) + 3z(0) + 2z(1) <= 4
  // -1 <= z(0) <= 1
  // -1 <= z(1) <= 1
  auto z = prog_.NewContinuousVariables<2>();
  prog_.AddBoundingBoxConstraint(-1, 1, z);
  Eigen::Matrix2d Q;
  Q << 1, 1, 1, 4;
  VectorDecisionVariable<3> y(x_(1), z(0), z(1));
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1orQ2(
      &prog_, x_, Q, y, Eigen::Vector3d(2, 3, 2), -1, 4, Eigen::Vector2d(1, 0),
      1, true, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ2Test0) {
  // 1 <= x(0)² + x(1)² + x(0)x(1) + 2x(0) <= 4
  Eigen::Matrix2d Q;
  Q << 1, 0.5, 0.5, 1;
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1orQ2(
      &prog_, x_, Q, x_, Eigen::Vector2d(1, 0), 1, 4, Eigen::Vector2d(1, -1),
      0.5, false, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ2Test1) {
  // -1 <= x(0)² + 3x(1)² + x(0)x(1) + 2x(0) + 4x(1) <= 4
  Eigen::Matrix2d Q;
  Q << 1, 0.5, 0.5, 3;
  const auto c = GenerateCostDirection();
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1orQ2(
      &prog_, x_, Q, x_, Eigen::Vector2d(2, 4), 1, 4, Eigen::Vector2d(1, -1),
      0.5, false, c);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ2Test2) {
  // The original non-convex problem is infeasible.
  // 1 <= x(0)² + x(1)² <= 4
  // x(0) + x(1) >= 10
  Eigen::Matrix2d Q1;
  Q1 << 1, 0, 0, 1;
  prog_.AddLinearConstraint(x_(0) + x_(1) >= 10);

  AddRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog_, x_, Q1, Eigen::Matrix2d::Zero(), x_, Eigen::Vector2d::Zero(), 1,
      4, Eigen::Vector2d(0, 1), 1);

  auto result = Solve(prog_).get_solution_result();
  EXPECT_TRUE(result == SolutionResult::kInfeasible_Or_Unbounded ||
              result == SolutionResult::kInfeasibleConstraints);
}

TEST_F(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ2Test3) {
  // The variable y in the linear term is not the same as the variable x in the
  // quadratic term.
  // -1 <= x(0)² + 3x(1)² + x(0)x(1) + 2x(0) + 4x(1) + 3y(0) <= 4
  // -1 <= y(0) <= 1
  Eigen::Matrix2d Q;
  Q << 1, 0.5, 0.5, 3;
  const auto c = GenerateCostDirection();
  auto x_prime = prog_.NewContinuousVariables<1>();
  const VectorDecisionVariable<3> y(x_(0), x_(1), x_prime(0));
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1orQ2(
      &prog_, x_, Q, y, Eigen::Vector3d(2, 4, 3), 1, 4, Eigen::Vector2d(1, -1),
      0.5, false, c);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
