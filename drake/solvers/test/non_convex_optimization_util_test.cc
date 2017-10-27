#include "drake/solvers/non_convex_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

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
    const Eigen::Ref<const Eigen::VectorXd>& p, double lower_bound,
    double upper_bound,
    const Eigen::Ref<const Eigen::VectorXd>& linearization_point,
    double trust_region_gap) {
  const auto& x0 = linearization_point;
  const auto result = RelaxNonConvexQuadraticConstraintInTrustRegion(
      prog, x, Q1, Q2, p, lower_bound, upper_bound, linearization_point,
      trust_region_gap);
  Binding<LinearConstraint> linear_constraint = std::get<0>(result);
  EXPECT_EQ(std::get<1>(result).size(), 2);
  Binding<RotatedLorentzConeConstraint> rotated_lorentz_cone_constraint1 =
      std::get<1>(result)[0];
  Binding<RotatedLorentzConeConstraint> rotated_lorentz_cone_constraint2 =
      std::get<1>(result)[1];
  VectorDecisionVariable<2> z = std::get<2>(result);
  Vector3<symbolic::Expression> y =
      linear_constraint.constraint()->A() * linear_constraint.variables();
  Vector3<symbolic::Expression> y_expected;
  y_expected << z(0) - z(1) + p.dot(x), z(0) - 2 * x0.dot(Q1 * x),
      z(1) - 2 * x0.dot(Q2 * x);
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(PolynomialEqual(symbolic::Polynomial(y(i)),
                                symbolic::Polynomial(y_expected(i)), 1E-10) ||
                PolynomialEqual(symbolic::Polynomial(y(i)),
                                symbolic::Polynomial(-y_expected(i)), 1E-10));
  }
  const VectorX<symbolic::Expression> y_lorentz1 =
      rotated_lorentz_cone_constraint1.constraint()->A() *
          rotated_lorentz_cone_constraint1.variables() +
      rotated_lorentz_cone_constraint1.constraint()->b();
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(y_lorentz1(0) * y_lorentz1(1)),
                      symbolic::Polynomial(z(0)), 1E-10));
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(
                          y_lorentz1.tail(y_lorentz1.rows() - 2).squaredNorm()),
                      symbolic::Polynomial(x.dot(Q1 * x)), 1E-10));

  const VectorX<symbolic::Expression> y_lorentz2 =
      rotated_lorentz_cone_constraint2.constraint()->A() *
          rotated_lorentz_cone_constraint2.variables() +
      rotated_lorentz_cone_constraint2.constraint()->b();
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(y_lorentz2(0) * y_lorentz2(1)),
                      symbolic::Polynomial(z(1)), 1E-10));
  EXPECT_TRUE(
      PolynomialEqual(symbolic::Polynomial(
                          y_lorentz2.tail(y_lorentz2.rows() - 2).squaredNorm()),
                      symbolic::Polynomial(x.dot(Q2 * x)), 1E-10));
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, Test0) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  Eigen::Matrix2d Q1 = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d Q2;
  Q2 << 1, 0.2, 0.2, 1;
  Eigen::Vector2d p(1, 2);
  const double lower_bound{0};
  const double upper_bound{0.1};
  const Eigen::Vector2d linearization_point(1, 2);
  const double trust_region_gap{1};
  CheckRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog, x, Q1, Q2, p, lower_bound, upper_bound, linearization_point,
      trust_region_gap);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, Test1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
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
      &prog, x, Q1, Q2, p, lower_bound, upper_bound, linearization_point,
      trust_region_gap);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion,
           TestRuntimeError) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  Eigen::Matrix2d non_positive_Q;
  non_positive_Q << 1, 1.5, 1.5, 1;
  Eigen::Matrix2d positive_Q;
  positive_Q << 1, 0.2, 0.2, 1;
  EXPECT_THROW(RelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog, x, non_positive_Q, positive_Q, Eigen::Vector2d(1, 0),
                   -1, 0.1, Eigen::Vector2d(1, 2), 1),
               std::runtime_error);
  EXPECT_THROW(RelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog, x, positive_Q, non_positive_Q, Eigen::Vector2d(1, 0),
                   -1, 0.1, Eigen::Vector2d(1, 2), 1),
               std::runtime_error);
  EXPECT_THROW(RelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog, x, positive_Q, positive_Q, Eigen::Vector2d(1, 0), -1,
                   0.1, Eigen::Vector2d(1, 2), -0.1),
               std::runtime_error);
}

void SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p, double lower_bound,
    double upper_bound, const Eigen::Ref<const Eigen::VectorXd>& x0,
    double trust_region_gap, const Eigen::Ref<const Eigen::MatrixXd>& c) {
  const auto relaxed_constraints =
      RelaxNonConvexQuadraticConstraintInTrustRegion(
          prog, x, Q1, Q2, p, lower_bound, upper_bound, x0, trust_region_gap);
  VectorDecisionVariable<2> z = std::get<2>(relaxed_constraints);

  auto cost = prog->AddLinearCost(x.cast<symbolic::Expression>().sum());
  for (int i = 0; i < c.cols(); ++i) {
    cost.constraint()->UpdateCoefficients(c.col(i).transpose());
    auto result = prog->Solve();
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    auto x_sol = prog->GetSolution(x);
    auto z_sol = prog->GetSolution(z);
    const double check_tol{1E-5};
    EXPECT_GE(z_sol(0) - z_sol(1) + p.dot(x_sol), lower_bound - check_tol);
    EXPECT_LE(z_sol(0) - z_sol(1) + p.dot(x_sol), upper_bound + check_tol);
    EXPECT_LE(z_sol(0), 2 * x0.dot(Q1 * x_sol) - x0.dot(Q1 * x0) +
                            trust_region_gap + check_tol);
    EXPECT_LE(z_sol(1), 2 * x0.dot(Q2 * x_sol) - x0.dot(Q2 * x0) +
                            trust_region_gap + check_tol);
    EXPECT_GE(z_sol(0), x_sol.dot(Q1 * x_sol) - check_tol);
    EXPECT_GE(z_sol(1), x_sol.dot(Q2 * x_sol) - check_tol);
    const double original_constraint_value{x_sol.dot((Q1 - Q2) * x_sol) +
                                           p.dot(x_sol)};
    EXPECT_LE(original_constraint_value,
              upper_bound + trust_region_gap + check_tol);
    EXPECT_GE(original_constraint_value,
              lower_bound - trust_region_gap - check_tol);
  }
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, SolveProblem0) {
  // For a problem
  // max c'x
  // s.t 1 <= x(0)² - x(1)² <= 2
  // The relaxation of the constraint is
  // 1 <= z(0) - z(1) <= 2
  // z(0) <= 2x₀(0)x(0) - x₀(0)² + d
  // z(1) <= 2x₀(1)x(1) - x₀(1)² + d
  // z(0) >= x(0)²
  // z(1) >= x(1)²
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  Eigen::Matrix2d Q1, Q2;
  Q1 << 1, 0, 0, 0;
  Q2 << 0, 0, 0, 1;
  // We linearize it about x = (2, 1). If we set the violation d to 0.5, within
  // the trust region, there should be a solution.
  const double trust_region_gap{0.5};
  const double lb{1};
  const double ub{2};
  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
       0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog, x, Q1, Q2, Eigen::Vector2d::Zero(), lb, ub, Eigen::Vector2d(2, 1),
      trust_region_gap, c);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, SolveProblem1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  Eigen::Matrix2d Q1, Q2;
  Q1 << 1, 0, 0, 2;
  Q2 << 2, 0.1, 0.1, 1;
  const double trust_region_gap{0.5};
  const double lb{1};
  const double ub{2};
  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
      0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog, x, Q1, Q2, Eigen::Vector2d(1, 3), lb, ub, Eigen::Vector2d(2, 1),
      trust_region_gap, c);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion,
           SolveInfeasibleProblem0) {
  // The non-convex problem is infeasible. With small relaxation gap, the
  // relaxed problem should also be infeasible.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // x(0) = 0
  // x(0)² - x(1)² = 1
  prog.AddBoundingBoxConstraint(0, 0, x(0));
  const Eigen::Matrix2d Q1 = Eigen::Vector2d(1, 0).asDiagonal();
  const Eigen::Matrix2d Q2 = Eigen::Vector2d(0, 1).asDiagonal();
  RelaxNonConvexQuadraticConstraintInTrustRegion(&prog, x, Q1, Q2,
                                                 Eigen::Vector2d::Zero(), 1, 1,
                                                 Eigen::Vector2d(1, 0), 0.1);

  auto result = prog.Solve();
  EXPECT_TRUE(result == SolutionResult::kInfeasible_Or_Unbounded ||
              result == SolutionResult::kInfeasibleConstraints);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion,
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
      RelaxNonConvexQuadraticConstraintInTrustRegion(
          &prog1, x1, Q1, Q2, Eigen::Vector2d::Zero(), 1, 1,
          Eigen::Vector2d(-5, -3), 0.1);

  auto result = prog1.Solve();
  EXPECT_TRUE(result == SolutionResult::kInfeasibleConstraints ||
              result == SolutionResult::kInfeasible_Or_Unbounded);

  // If we linearize the problem at about (7, -5), then the relaxed problem has
  // a solution around the linearization point.
  MathematicalProgram prog2;
  auto x2 = prog2.NewContinuousVariables<2>();
  prog2.AddLinearConstraint(x2(0) >= x2(1) + 2);
  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
      0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  SolveRelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog2, x2, Q1, Q2, Eigen::Vector2d::Zero(), 1, 1, Eigen::Vector2d(7, -5),
      1.5, c);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Q2) {
  // Should throw a runtime error.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // Both Q1 and Q2 are 0
  EXPECT_THROW(RelaxNonConvexQuadraticConstraintInTrustRegion(
                   &prog, x, Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Zero(),
                   Eigen::Vector2d(1, 2), 1, 2, Eigen::Vector2d(2, 1), 1),
               std::runtime_error);
  // Q1 is zero and upper_bound is infinity.
  EXPECT_THROW(
      RelaxNonConvexQuadraticConstraintInTrustRegion(
          &prog, x, Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Identity(),
          Eigen::Vector2d(1, 2), 1, std::numeric_limits<double>::infinity(),
          Eigen::Vector2d(2, 1), 1),
      std::runtime_error);
  // Q2 is zero and lower_bound is -infinity.
  EXPECT_THROW(
      RelaxNonConvexQuadraticConstraintInTrustRegion(
          &prog, x, Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Zero(),
          Eigen::Vector2d(1, 2), -std::numeric_limits<double>::infinity(), 1,
          Eigen::Vector2d(2, 1), 1),
      std::runtime_error);
}

void SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p, double lb, double ub,
    const Eigen::Ref<const Eigen::VectorXd>& x0, double trust_region_gap,
    const Eigen::Ref<const Eigen::MatrixXd>& c) {
  const auto relaxed_constraints =
      RelaxNonConvexQuadraticConstraintInTrustRegion(
          prog, x, Eigen::MatrixXd::Zero(x.rows(), x.rows()), Q2, p, lb, ub, x0,
          trust_region_gap);
  const Binding<LinearConstraint> linear_constraint =
      std::get<0>(relaxed_constraints);
  EXPECT_EQ(std::get<1>(relaxed_constraints).size(), 1);
  const Binding<RotatedLorentzConeConstraint> lorentz_cone1 =
      std::get<1>(relaxed_constraints)[0];
  const VectorDecisionVariable<1> z = std::get<2>(relaxed_constraints);
  Vector2<symbolic::Expression> linear_expr, linear_expr_expected;
  linear_expr_expected << 2 * x0.dot(Q2 * x) - z(0), p.dot(x) - z(0);
  linear_expr =
      linear_constraint.constraint()->A() * linear_constraint.variables();
  Eigen::Vector2d linear_lb, linear_ub;
  linear_lb << x0.dot(Q2 * x0) - trust_region_gap, lb;
  linear_ub << std::numeric_limits<double>::infinity(), ub;
  for (int i = 0; i < 2; ++i) {
    EXPECT_TRUE(PolynomialEqual(symbolic::Polynomial(linear_expr(i)),
                                symbolic::Polynomial(linear_expr_expected(i)),
                                1E-10));
    EXPECT_TRUE(CompareMatrices(linear_constraint.constraint()->lower_bound(),
                                linear_lb, 1E-15, MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(linear_constraint.constraint()->upper_bound(),
                                linear_ub, 1E-15, MatrixCompareType::absolute));
  }

  VectorX<symbolic::Expression> y1 =
      lorentz_cone1.constraint()->A() * lorentz_cone1.variables() +
      lorentz_cone1.constraint()->b();
  EXPECT_TRUE(PolynomialEqual(symbolic::Polynomial(y1(0) * y1(1)),
                              symbolic::Polynomial(z(0)), 1E-15));
  EXPECT_TRUE(PolynomialEqual(
      symbolic::Polynomial(y1.tail(y1.rows() - 2).squaredNorm()),
      symbolic::Polynomial(x.dot(Q2 * x)), 1E-15));

  auto cost = prog->AddLinearCost(x.cast<symbolic::Expression>().sum());
  for (int i = 0; i < c.cols(); ++i) {
    cost.constraint()->UpdateCoefficients(c.col(i).transpose());
    auto result = prog->Solve();
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    const double z_sol = prog->GetSolution(z(0));
    const Eigen::Vector2d x_sol = prog->GetSolution(x);
    const double tol{1E-6};
    EXPECT_GE(-z_sol + p.dot(x_sol), lb - tol);
    EXPECT_LE(-z_sol + p.dot(x_sol), ub + tol);
    EXPECT_LE(z_sol, 2 * x0.dot(Q2 * x_sol) - x0.dot(Q2 * x0) +
                         trust_region_gap + tol);
    EXPECT_GE(z_sol, x_sol.dot(Q2 * x_sol) - tol);
    const double original_constraint_val{-x_sol.dot(Q2 * x_sol) + p.dot(x_sol)};
    EXPECT_GE(original_constraint_val, lb - trust_region_gap - tol);
    EXPECT_LE(original_constraint_val, ub + trust_region_gap + tol);
  }
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Test0) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // -x(0)² - x(1)² - x(0)x(1) + 2*x(0) <= 1
  Eigen::Matrix2d Q2;
  Q2 << 1, 0.5, 0.5, 1;
  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
      0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1(
      &prog, x, Q2, Eigen::Vector2d(2, 0),
      -std::numeric_limits<double>::infinity(), 1, Eigen::Vector2d(1, 2), 1, c);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Test1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // -1 <= -x(0)² - 4x(1)² - 2x(0)x(1) + 3x(0) + 2x(1) <= 4
  Eigen::Matrix2d Q2;
  Q2 << 1, 1, 1, 4;
  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
      0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ1(
      &prog, x, Q2, Eigen::Vector2d(3, 2), -1, 4, Eigen::Vector2d(1, 0), 1, c);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ1Test2) {
  // The original non-convex problem is infeasible.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // -4 <= -x(0)² - x(1)² <= -1
  // x(0) + x(1) >= 10
  Eigen::Matrix2d Q2;
  Q2 << 1, 0, 0, 1;
  prog.AddLinearConstraint(x(0) + x(1) >= 10);

  RelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog, x, Eigen::Matrix2d::Zero(), Q2, Eigen::Vector2d::Zero(), -4, -1,
      Eigen::Vector2d(0, 1), 1);

  auto result = prog.Solve();
  EXPECT_TRUE(result == SolutionResult::kInfeasible_Or_Unbounded ||
              result == SolutionResult::kInfeasibleConstraints);
}

void SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ2(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::VectorXd>& p, double lb, double ub,
    const Eigen::Ref<const Eigen::VectorXd>& x0, double trust_region_gap,
    const Eigen::Ref<const Eigen::MatrixXd>& c) {
  const auto relaxed_constraints =
      RelaxNonConvexQuadraticConstraintInTrustRegion(
          prog, x, Q1, Eigen::MatrixXd::Zero(x.rows(), x.rows()), p, lb, ub, x0,
          trust_region_gap);
  const Binding<LinearConstraint> linear_constraint =
      std::get<0>(relaxed_constraints);
  EXPECT_EQ(std::get<1>(relaxed_constraints).size(), 1);
  const Binding<RotatedLorentzConeConstraint> lorentz_cone1 =
      std::get<1>(relaxed_constraints)[0];
  const VectorDecisionVariable<1> z = std::get<2>(relaxed_constraints);
  Vector2<symbolic::Expression> linear_expr, linear_expr_expected;
  linear_expr_expected << 2 * x0.dot(Q1 * x) - z(0), p.dot(x) + z(0);
  linear_expr =
      linear_constraint.constraint()->A() * linear_constraint.variables();
  Eigen::Vector2d linear_lb, linear_ub;
  linear_lb << x0.dot(Q1 * x0) - trust_region_gap, lb;
  linear_ub << std::numeric_limits<double>::infinity(), ub;
  const double polynomial_tol{1E-14};
  for (int i = 0; i < 2; ++i) {
    EXPECT_TRUE(PolynomialEqual(symbolic::Polynomial(linear_expr(i)),
                                symbolic::Polynomial(linear_expr_expected(i)),
                                polynomial_tol));
    EXPECT_TRUE(CompareMatrices(linear_constraint.constraint()->lower_bound(),
                                linear_lb, 1E-15, MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(linear_constraint.constraint()->upper_bound(),
                                linear_ub, 1E-15, MatrixCompareType::absolute));
  }

  VectorX<symbolic::Expression> y1 =
      lorentz_cone1.constraint()->A() * lorentz_cone1.variables() +
      lorentz_cone1.constraint()->b();
  EXPECT_TRUE(PolynomialEqual(symbolic::Polynomial(y1(0) * y1(1)),
                              symbolic::Polynomial(z(0)), polynomial_tol));
  EXPECT_TRUE(PolynomialEqual(
      symbolic::Polynomial(y1.tail(y1.rows() - 2).squaredNorm()),
      symbolic::Polynomial(x.dot(Q1 * x)), polynomial_tol));

  auto cost = prog->AddLinearCost(x.cast<symbolic::Expression>().sum());
  for (int i = 0; i < c.cols(); ++i) {
    cost.constraint()->UpdateCoefficients(c.col(i).transpose());
    auto result = prog->Solve();
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    const double z_sol = prog->GetSolution(z(0));
    const Eigen::Vector2d x_sol = prog->GetSolution(x);
    const double tol{1E-6};
    EXPECT_GE(z_sol + p.dot(x_sol), lb - tol);
    EXPECT_LE(z_sol + p.dot(x_sol), ub + tol);
    EXPECT_LE(z_sol, 2 * x0.dot(Q1 * x_sol) - x0.dot(Q1 * x0) +
                         trust_region_gap + tol);
    EXPECT_GE(z_sol, x_sol.dot(Q1 * x_sol) - tol);
    const double original_constraint_val{x_sol.dot(Q1 * x_sol) + p.dot(x_sol)};
    EXPECT_GE(original_constraint_val, lb - trust_region_gap - tol);
    EXPECT_LE(original_constraint_val, ub + trust_region_gap + tol);
  }
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ2Test0) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // 1 <= x(0)² + x(1)² + x(0)x(1) + 2x(0) <= 4
  Eigen::Matrix2d Q1;
  Q1 << 1, 0.5, 0.5, 1;

  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
      0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ2(
      &prog, x, Q1, Eigen::Vector2d(1, 0), 1, 4, Eigen::Vector2d(1, -1), 0.5,
      c);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ2Test1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // -1 <= x(0)² + 3x(1)² + x(0)x(1) + 2x(0) + 4x(1) <= 4
  Eigen::Matrix2d Q1;
  Q1 << 1, 0.5, 0.5, 3;

  Eigen::Matrix<double, 2, 10> c;
  // clang-format off
  c << 1, 1,  1, 0, 0, 0, -1, -1, -1, 1,
       0, -1, 1, 1, 0, -1, 1, 0,  -1, 2;
  // clang-format on
  SolveRelaxNonConvexQuadraticConstraintInTrustRegionWithZeroQ2(
      &prog, x, Q1, Eigen::Vector2d(2, 4), 1, 4, Eigen::Vector2d(1, -1), 0.5,
      c);
}

GTEST_TEST(TestRelaxNonConvexQuadraticConstraintInTrustRegion, ZeroQ2Test2) {
  // The original non-convex problem is infeasible.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // 1 <= x(0)² + x(1)² <= 4
  // x(0) + x(1) >= 10
  Eigen::Matrix2d Q1;
  Q1 << 1, 0, 0, 1;
  prog.AddLinearConstraint(x(0) + x(1) >= 10);

  RelaxNonConvexQuadraticConstraintInTrustRegion(
      &prog, x, Q1, Eigen::Matrix2d::Zero(), Eigen::Vector2d::Zero(), 1, 4,
      Eigen::Vector2d(0, 1), 1);

  auto result = prog.Solve();
  EXPECT_TRUE(result == SolutionResult::kInfeasible_Or_Unbounded ||
      result == SolutionResult::kInfeasibleConstraints);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
