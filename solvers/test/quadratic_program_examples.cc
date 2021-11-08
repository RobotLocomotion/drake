#include "drake/solvers/test/quadratic_program_examples.h"

#include <limits>
#include <optional>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::RowVector2d;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using std::numeric_limits;
using drake::symbolic::Expression;

using ::testing::HasSubstr;

namespace drake {
namespace solvers {
namespace test {

QuadraticProgramTest::QuadraticProgramTest() {
  auto cost_form = std::get<0>(GetParam());
  auto constraint_form = std::get<1>(GetParam());
  switch (std::get<2>(GetParam())) {
    case QuadraticProblems::kQuadraticProgram0 : {
      prob_ = std::make_unique<QuadraticProgram0>(cost_form, constraint_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram1 : {
      prob_ = std::make_unique<QuadraticProgram1>(cost_form, constraint_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram2 : {
      prob_ = std::make_unique<QuadraticProgram2>(cost_form, constraint_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram3 : {
      prob_ = std::make_unique<QuadraticProgram3>(cost_form, constraint_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram4 : {
      prob_ = std::make_unique<QuadraticProgram4>(cost_form, constraint_form);
      break;
    }
    default : throw std::runtime_error("Un-recognized quadratic problem.");
  }
}

std::vector<QuadraticProblems> quadratic_problems() {
  return std::vector<QuadraticProblems>{QuadraticProblems::kQuadraticProgram0,
                                        QuadraticProblems::kQuadraticProgram1,
                                        QuadraticProblems::kQuadraticProgram2,
                                        QuadraticProblems::kQuadraticProgram3,
                                        QuadraticProblems::kQuadraticProgram4};
}

QuadraticProgram0::QuadraticProgram0(CostForm cost_form,
                                     ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form),
      x_(),
      x_expected_(0.25, 0.75) {
  x_ = prog()->NewContinuousVariables<2>();
  switch (cost_form) {
    case CostForm::kNonSymbolic : {
      Matrix2d Q;
      // clang-format off
      Q << 4, 2,
          0, 2;
      // clang-format on
      const Vector2d b(1, 0);
      const double c = 3;
      prog()->AddQuadraticCost(Q, b, c, x_);
      prog()->AddLinearCost(Vector1d(1.0), x_.segment<1>(1));
      break;
    }
    case CostForm::kSymbolic : {
      prog()->AddQuadraticCost(2 * x_(0) * x_(0) + x_(0) * x_(1) +
                               x_(1) * x_(1) + x_(0) + x_(1) + 3);
      break;
    }
    default: {
      throw std::runtime_error("Unsupported cost form.");
    }
  }
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      prog()->AddBoundingBoxConstraint(-1, numeric_limits<double>::infinity(),
                                       x_(1));
      prog()->AddLinearEqualityConstraint(RowVector2d(1, 1), 1, x_);
      break;
    }
    case ConstraintForm::kSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      prog()->AddLinearEqualityConstraint(x_(0) + x_(1), 1);
      break;
    }
    case ConstraintForm::kFormula : {
      prog()->AddLinearConstraint(x_(0) >= 0);
      prog()->AddLinearConstraint(x_(1) >= 0);
      prog()->AddLinearConstraint(x_(0) + x_(1) == 1);
      break;
    }
    default : {
      throw std::runtime_error("Unsupported constraint form.");
    }
  }
}

void QuadraticProgram0::CheckSolution(
    const MathematicalProgramResult& result) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(result.get_solver_id());
  if (result.get_solver_id() == GurobiSolver::id()) {
    tol = 1E-8;
  } else if (result.get_solver_id() == MosekSolver::id()) {
    // TODO(hongkai.dai): the default parameter in Mosek 8 generates low
    // accuracy solution. We should set the accuracy tolerance
    // MSK_DPARAM_INTPNT_QO_REL_TOL_GAP to 1E-10 to improve the accuracy.
    tol = 3E-5;
  }
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

QuadraticProgram1::QuadraticProgram1(CostForm cost_form,
                                     ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form),
      x_{},
      x_expected_(0, 1, 2.0 / 3) {
  x_ = prog()->NewContinuousVariables<3>();
  switch (cost_form) {
    case CostForm::kNonSymbolic : {
      Matrix3d Q = 2 * Matrix3d::Identity();
      Q(0, 1) = 1;
      Q(1, 2) = 1;
      Q(1, 0) = 1;
      Q(2, 1) = 1;
      Vector3d b{};
      b << 2.0, 0.0, 0.0;
      prog()->AddQuadraticCost(Q, b, x_);
      break;
    }
    case CostForm::kSymbolic : {
      prog()->AddQuadraticCost(x_(0) * x_(0) + x_(0) * x_(1) + x_(1) * x_(1) +
                               x_(1) * x_(2) + x_(2) * x_(2) + 2 * x_(0));
      break;
    }
    default : throw std::runtime_error("Unsupported cost form.");
  }
  Vector4d b_lb(4, -numeric_limits<double>::infinity(), -20,
                -numeric_limits<double>::infinity());
  Vector4d b_ub(numeric_limits<double>::infinity(), -1, 100,
                numeric_limits<double>::infinity());
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      Eigen::Matrix<double, 4, 3> A1;
      A1 << 1, 2, 3, -1, -1, 0, 0, 1, 2, 1, 1, 2;

      prog()->AddLinearConstraint(A1, b_lb, b_ub, x_);
      // This test also handles linear equality constraint
      prog()->AddLinearEqualityConstraint(Eigen::RowVector3d(3, 1, 3), 3, x_);
      break;
    }
    case ConstraintForm::kSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      Vector4<Expression> expr;
      // clang-format off
      expr << x_(0) + 2 * x_(1) + 3 * x_(2),
          -x_(0) - x_(1),
          x_(1) + 2 * x_(2),
          x_(0) + x_(1) + 2 * x_(2);
      // clang-format on
      prog()->AddLinearConstraint(expr, b_lb, b_ub);
      prog()->AddLinearEqualityConstraint(3 * x_(0) + x_(1) + 3 * x_(2), 3);
      break;
    }
    case ConstraintForm::kFormula : {
      for (int i = 0; i < 3; ++i) {
        prog()->AddLinearConstraint(x_(i) >= 0);
      }
      prog()->AddLinearConstraint(x_(0) + 2 * x_(1) + 3 * x_(2) >= 4);
      prog()->AddLinearConstraint(-x_(0) - x_(1) <= -1);
      prog()->AddLinearConstraint(x_(1) + 2 * x_(2), -20, 100);
      // TODO(hongkai.dai): Uncomment the next line, when we resolve the error
      // with infinity on the right handside of a formula.
      // prog()->AddLinearConstraint(x_(0) + x_(1) + 2 * x_(2) <=
      // numeric_limits<double>::infinity());
      prog()->AddLinearConstraint(3 * x_(0) + x_(1) + 3 * x_(2) == 3);
      break;
    }
    default : throw std::runtime_error("Unsupported constraint form.");
  }
}

void QuadraticProgram1::CheckSolution(
    const MathematicalProgramResult& result) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(result.get_solver_id());
  if (result.get_solver_id() == GurobiSolver::id()) {
    tol = 1E-8;
  } else if (result.get_solver_id() == MosekSolver::id()) {
    tol = 1E-7;
  }
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

QuadraticProgram2::QuadraticProgram2(CostForm cost_form,
                                     ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form), x_{}, x_expected_{} {
  x_ = prog()->NewContinuousVariables<5>();

  Eigen::Matrix<double, 5, 1> Q_diag{};
  Q_diag << 5.5, 6.5, 6.0, 5.3, 7.5;
  Eigen::Matrix<double, 5, 5> Q = Q_diag.asDiagonal();
  Q(2, 3) = 0.2;

  Eigen::Matrix<double, 5, 1> b;
  b << 3.2, 1.3, 5.6, 9.0, 1.2;
  switch (cost_form) {
    case CostForm::kNonSymbolic : {
      prog()->AddQuadraticCost(Q, b, x_);
      break;
    }
    case CostForm::kSymbolic : {
      prog()->AddQuadraticCost(0.5 * x_.dot(Q * x_) + b.dot(x_));
      break;
    }
    default : throw std::runtime_error("Unsupported cost form.");
  }

  Eigen::Matrix<double, 5, 5> Q_symmetric = 0.5 * (Q + Q.transpose());
  x_expected_ = -Q_symmetric.llt().solve(b);
}

void QuadraticProgram2::CheckSolution(
    const MathematicalProgramResult& result) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(result.get_solver_id());
  if (result.get_solver_id() == MosekSolver::id()) {
    tol = 1E-8;
  } else if (result.get_solver_id() == SnoptSolver::id()) {
    tol = 1E-6;
  } else if (result.get_solver_id() == ClpSolver::id()) {
    tol = 2E-8;
  }
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

QuadraticProgram3::QuadraticProgram3(CostForm cost_form,
                                     ConstraintForm constraint_form)
: OptimizationProgram(cost_form, constraint_form), x_{}, x_expected_{} {
  x_ = prog()->NewContinuousVariables<6>();
  Vector4d Q1_diag;
  Q1_diag << 5.5, 6.5, 6.0, 7.0;
  Eigen::Matrix4d Q1 = Q1_diag.asDiagonal();
  Q1(1, 2) = 0.1;

  Vector4d Q2_diag;
  Q2_diag << 7.0, 2.2, 1.1, 1.3;
  Matrix4d Q2 = Q2_diag.asDiagonal();
  Q2(0, 2) = -0.02;

  Vector4d b1(3.1, -1.4, -5.6, 0.6);
  Vector4d b2(2.3, -5.8, 6.7, 2.3);

  switch (cost_form) {
    case CostForm::kNonSymbolic : {
      prog()->AddQuadraticCost(Q1, b1, x_.head<4>());
      prog()->AddQuadraticCost(Q2, b2, x_.tail<4>());
      break;
    }
    case CostForm::kSymbolic : {
      prog()->AddQuadraticCost(
          0.5 * x_.head<4>().dot(Q1 * x_.head<4>()) + b1.dot(x_.head<4>()) +
          0.5 * x_.tail<4>().dot(Q2 * x_.tail<4>()) + b2.dot(x_.tail<4>()));
      break;
    }
    default : throw std::runtime_error("Unsupported cost form.");
  }

  Eigen::Matrix<double, 6, 6> Q;
  Q.setZero();
  Q.topLeftCorner<4, 4>() = Q1;
  Q.bottomRightCorner<4, 4>() += Q2;
  Eigen::Matrix<double, 6, 6> Q_symmetric = 0.5 * (Q + Q.transpose());

  Vector6d b;
  b.setZero();
  b.head<4>() = b1;
  b.tail<4>() += b2;

  x_expected_ = -Q_symmetric.llt().solve(b);
}

void QuadraticProgram3::CheckSolution(
    const MathematicalProgramResult& result) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(result.get_solver_id());
  if (result.get_solver_id() == MosekSolver::id()) {
    tol = 1E-8;
  }
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

QuadraticProgram4::QuadraticProgram4(CostForm cost_form,
                                     ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form),
      x_{},
      x_expected_(0.8, 0.2, 0.6) {
  x_ = prog()->NewContinuousVariables<3>();
  switch (cost_form) {
    case CostForm::kNonSymbolic : {
      Matrix3d Q = Matrix3d::Identity();
      Q(2, 2) = 2.0;
      Vector3d b = Vector3d::Zero();
      prog()->AddQuadraticCost(2 * Q, b, x_);
      break;
    }
    case CostForm::kSymbolic : {
      prog()->AddQuadraticCost(x_(0) * x_(0) + x_(1) * x_(1) +
                               2 * x_(2) * x_(2));
      break;
    }
    default : throw std::runtime_error("Unsupported cost form.");
  }
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic : {
      prog()->AddLinearEqualityConstraint(RowVector2d(1, 1), 1, x_.head<2>());
      prog()->AddLinearEqualityConstraint(RowVector2d(1, 2), 2,
                                          {x_.segment<1>(0), x_.segment<1>(2)});
      break;
    }
    case ConstraintForm::kSymbolic : {
      prog()->AddLinearEqualityConstraint(x_(0) + x_(1), 1);
      prog()->AddLinearEqualityConstraint(x_(0) + 2 * x_(2), 2);
      break;
    }
    case ConstraintForm::kFormula : {
      prog()->AddLinearConstraint(x_(0) + x_(1) == 1);
      prog()->AddLinearConstraint(x_(0) + 2 * x_(2) == 2);
      break;
    }
    default : throw std::runtime_error("Unsupported constraint form.");
  }
}

void QuadraticProgram4::CheckSolution(
    const MathematicalProgramResult& result) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(result.get_solver_id());
  if (result.get_solver_id() == MosekSolver::id()) {
    tol = 1E-8;
  }
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

void TestQPonUnitBallExample(const SolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Eigen::Vector2d x_desired;
  x_desired << 1.0, 0.0;
  auto objective = prog.AddQuadraticErrorCost(Q, x_desired, x).evaluator();

  Eigen::Matrix2d A;
  A << 1.0, 1.0, -1.0, 1.0;
  Eigen::Vector2d ub = Eigen::Vector2d::Constant(1.0);
  Eigen::Vector2d lb = Eigen::Vector2d::Constant(-1.0);
  auto constraint = prog.AddLinearConstraint(A, lb, ub, x).evaluator();
  Eigen::Vector2d x_expected;

  const int N = 40;  // number of points to test around the circle
  for (int i = 0; i < N; i++) {
    double theta = 2.0 * M_PI * i / N;
    x_desired << sin(theta), cos(theta);
    objective->UpdateCoefficients(2.0 * Q, -2.0 * Q * x_desired);

    if (theta <= M_PI_2) {
      // simple lagrange multiplier problem:
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x+y=1
      x_expected << (x_desired(0) - x_desired(1) + 1.0) / 2.0,
          (x_desired(1) - x_desired(0) + 1.0) / 2.0;
    } else if (theta <= M_PI) {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x-y=1
      x_expected << (x_desired(0) + x_desired(1) + 1.0) / 2.0,
          (x_desired(0) + x_desired(1) - 1.0) / 2.0;
    } else if (theta <= 3.0 * M_PI_2) {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x+y=-1
      x_expected << (x_desired(0) - x_desired(1) - 1.0) / 2.0,
          (x_desired(1) - x_desired(0) - 1.0) / 2.0;
    } else {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x-y=-1
      x_expected << (x_desired(0) + x_desired(1) - 1.0) / 2.0,
          (x_desired(0) + x_desired(1) + 1.0) / 2.0;
    }

    std::optional<Eigen::VectorXd> initial_guess;
    if (solver.solver_id() == SnoptSolver::id()) {
      initial_guess.emplace(Eigen::VectorXd::Zero(2));
    }
    const MathematicalProgramResult result =
        RunSolver(prog, solver, initial_guess);
    const auto& x_value = result.GetSolution(x);

    double tol = 1E-4;
    if (result.get_solver_id() == MosekSolver::id()) {
      // Regression from MOSEK 8.1 to MOSEK 9.2.
      tol = 2E-4;
    }
    EXPECT_TRUE(CompareMatrices(x_value, x_expected, tol,
                                MatrixCompareType::absolute));
  }

  // provide some test coverage for changing Q
  //
  {
    // now 2(x-xd)^2 + (y-yd)^2 s.t. x+y=1
    x_desired << 1.0, 1.0;
    Q(0, 0) = 2.0;
    objective->UpdateCoefficients(2.0 * Q, -2.0 * Q * x_desired);

    x_expected << 2.0 / 3.0, 1.0 / 3.0;

    prog.SetSolverOption(GurobiSolver::id(), "BarConvTol", 1E-9);
    // The default accuracy in SCS isn't enough, we set it to 1E-6.
    prog.SetSolverOption(ScsSolver::id(), "eps_abs", 1E-6);
    prog.SetSolverOption(ScsSolver::id(), "eps_rel", 1E-6);
    MathematicalProgramResult result;
    ASSERT_NO_THROW(result = RunSolver(prog, solver));

    const auto& x_value = result.GetSolution(x);
    EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1e-5,
                                MatrixCompareType::absolute));
    ExpectSolutionCostAccurate(prog, result, 1E-5);
  }
}

void TestQPDualSolution1(const SolverInterface& solver,
                         const SolverOptions& solver_options, double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto constraint1 = prog.AddLinearConstraint(2 * x[0] + 3 * x[1], -2, 3);
  auto constraint2 = prog.AddLinearEqualityConstraint(x[1] + 4 * x[2] == 3);
  auto constraint3 = prog.AddBoundingBoxConstraint(0, 3, x);
  prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] * x[1] + x[2] * x[2]);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, solver_options, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x),
                                Eigen::Vector3d(0, 1. / 11, 8. / 11.), tol));
    // At the optimal solution, the active constraints are
    // x[0] >= 0
    // x[1] + 4 * x[2] == 3
    // Solving the KKT condition, we get the dual solution as
    // dual solution for x[0] >= 0 is 0
    // dual solution for x[1] + 4 * x[2] == 3 is 0.363636
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint1),
                                Vector1d(0.), tol));
    const double dual_solution_expected = 0.363636;
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint2),
                                Vector1d(dual_solution_expected), tol));
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint3),
                                Eigen::Vector3d::Zero(), tol));

    // Now update the equality constraint right-hand side with a small amount,
    // check the optimal cost of the updated QP. The change in the optimal cost
    // should be equal to dual variable solution.
    const double delta = 1e-5;
    constraint2.evaluator()->set_bounds(Vector1d(3 + delta),
                                        Vector1d(3 + delta));
    MathematicalProgramResult result_updated;
    solver.Solve(prog, {}, solver_options, &result_updated);
    EXPECT_NEAR(
        (result_updated.get_optimal_cost() - result.get_optimal_cost()) / delta,
        dual_solution_expected, tol);
  }
}

void TestQPDualSolution2(const SolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto constraint1 = prog.AddLinearConstraint(2 * x[0] + 4 * x[1], 0, 3);
  auto constraint2 = prog.AddLinearEqualityConstraint(x[1] - 4 * x[2] == -2);
  auto constraint3 = prog.AddBoundingBoxConstraint(-1, 2, x);
  prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] * x[1] + x[2] * x[2] +
                        2 * x[1] * x[2] + 4 * x[2]);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    // At the optimal solution, the active constraints are
    // 2 * x[0] + 4 * x[1] >= 0
    // x[1] - 4 * x[2] == -2
    // Solving the KKT condition, we get the dual solution as
    // dual solution for 2 * x[0] + 4 * x[1] >= 0 is 0.34285714
    // dual solution for x[1] - 4 * x[2] == -2 is -1.14285714
    const double dual_solution_expected = 0.34285714;
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint1),
                                Vector1d(dual_solution_expected), 1e-6));
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint2),
                                Vector1d(-1.14285714), 1e-6));
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint3),
                                Eigen::Vector3d::Zero(), 1e-6));
    // Now perturb the lower bound of the inequality by a little bit, the change
    // of the optimal cost should be equal to the dual solution.
    const double delta = 1e-5;
    constraint1.evaluator()->UpdateLowerBound(Vector1d(delta));
    MathematicalProgramResult result_updated;
    solver.Solve(prog, {}, {}, &result_updated);
    EXPECT_NEAR(
        (result_updated.get_optimal_cost() - result.get_optimal_cost()) / delta,
        dual_solution_expected, 1e-5);
  }
}

void TestQPDualSolution3(const SolverInterface& solver, double tol,
                         double sensitivity_tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto constraint = prog.AddBoundingBoxConstraint(-1, 2, x);
  prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] * x[1] + 2 * x[0] * x[1] -
                        8 * x[0] + 6 * x[1]);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(x), Eigen::Vector2d(2, -1), tol));
    // At the optimal solution, the active constraints are
    // x[0] <= 2
    // x[1] >= -1
    // Solving the KKT condition, we get the dual solution as
    // dual solution for x[0] <= 2 is -6
    // dual solution for x[1] >= -1 is 6
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint),
                                Eigen::Vector2d(-6, 6), tol));
    // Now perturb the bounds a bit, the change of the optimal cost should match
    // with the dual solution.
    const double delta = 1e-5;
    constraint.evaluator()->UpdateUpperBound(Eigen::Vector2d(2 + delta, 2));
    MathematicalProgramResult result1;
    solver.Solve(prog, {}, {}, &result1);
    EXPECT_NEAR(
        (result1.get_optimal_cost() - result.get_optimal_cost()) / delta, -6,
        sensitivity_tol);
    constraint.evaluator()->UpdateUpperBound(Eigen::Vector2d(2, 2 + delta));
    MathematicalProgramResult result2;
    solver.Solve(prog, {}, {}, &result2);
    // The dual solution for x[1] <= 2 is 0.
    EXPECT_NEAR(result2.get_optimal_cost(), result.get_optimal_cost(),
                sensitivity_tol);

    constraint.evaluator()->set_bounds(Eigen::Vector2d(-1 + delta, -1),
                                       Eigen::Vector2d(2, 2));
    MathematicalProgramResult result3;
    solver.Solve(prog, {}, {}, &result3);
    // The dual solution for x[0] >= -1 is 0.
    EXPECT_NEAR(result3.get_optimal_cost(), result.get_optimal_cost(),
                sensitivity_tol);

    constraint.evaluator()->UpdateLowerBound(Eigen::Vector2d(-1, -1 + delta));
    MathematicalProgramResult result4;
    solver.Solve(prog, {}, {}, &result4);
    EXPECT_NEAR(
        (result4.get_optimal_cost() - result.get_optimal_cost()) / delta, 6,
        sensitivity_tol);

    // Now add more bounding box constraints (but with looser bounds than the -1
    // <= x <= 2 bound already imposed). The dual solution for these bounds
    // should be zero.
    constraint.evaluator()->set_bounds(Eigen::Vector2d(-1, -1),
                                       Eigen::Vector2d(2, 2));
    auto constraint1 = prog.AddBoundingBoxConstraint(-2, 3, x[0]);
    auto constraint2 = prog.AddBoundingBoxConstraint(-1.5, 3.5, x[1]);
    MathematicalProgramResult result5;
    solver.Solve(prog, {}, {}, &result5);
    EXPECT_TRUE(CompareMatrices(result5.GetDualSolution(constraint),
                                Eigen::Vector2d(-6, 6), tol));
    EXPECT_TRUE(
        CompareMatrices(result5.GetDualSolution(constraint1), Vector1d(0)));
    EXPECT_TRUE(
        CompareMatrices(result5.GetDualSolution(constraint2), Vector1d(0)));
  }
}

void TestEqualityConstrainedQPDualSolution1(const SolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto constraint = prog.AddLinearEqualityConstraint(x[0] + x[1] == 1);
  prog.AddQuadraticCost(x[0] * x[0] + x[1] * x[1]);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetDualSolution(constraint), Vector1d(1), 1e-5));
  }
}

void TestEqualityConstrainedQPDualSolution2(const SolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  Eigen::Matrix<double, 2, 3> A;
  A << 1, 0, 3, 0, 2, 1;
  auto constraint =
      prog.AddLinearEqualityConstraint(A, Eigen::Vector2d(1, 2), x);
  prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] * x[1] + x[2] * x[2] + 4 * x[1]);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(
        result.GetSolution(x),
        Eigen::Vector3d(-0.42857143, 0.76190476, 0.47619048), 1e-5));
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint),
                                Eigen::Vector2d(-0.85714286, 3.52380952),
                                1e-5));
  }
}

void TestNonconvexQP(const SolverInterface& solver, bool convex_solver,
                     double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto nonconvex_cost = prog.AddQuadraticCost(-x(0) * x(0) + x(1) * x(1) + 2);
  // Use a description that would never be mistaken as any other part of the
  // error message.
  const std::string description{"lorem ipsum"};
  nonconvex_cost.evaluator()->set_description(description);
  prog.AddBoundingBoxConstraint(0, 1, x);
  if (convex_solver) {
    EXPECT_FALSE(solver.AreProgramAttributesSatisfied(prog));
    EXPECT_THAT(solver.ExplainUnsatisfiedProgramAttributes(prog),
                HasSubstr("is non-convex"));
    EXPECT_THAT(solver.ExplainUnsatisfiedProgramAttributes(prog),
                HasSubstr(description));
  } else {
    MathematicalProgramResult result;
    // Use a non-zero initial guess, since at x = [0, 0], the gradient is 0.
    solver.Solve(prog, Eigen::Vector2d(0.1, 0.1), std::nullopt, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(x), Eigen::Vector2d(1, 0), tol));
    EXPECT_NEAR(result.get_optimal_cost(), 1., tol);
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
