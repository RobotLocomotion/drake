#include "drake/solvers/test/linear_program_examples.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Vector4d;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Matrix2d;
using std::numeric_limits;
using drake::symbolic::Expression;

namespace drake {
namespace solvers {
namespace test {
const double kInf = std::numeric_limits<double>::infinity();

LinearFeasibilityProgram::LinearFeasibilityProgram(
    ConstraintForm constraint_form)
    : OptimizationProgram(CostForm::kSymbolic, constraint_form), x_() {
  x_ = prog()->NewContinuousVariables<3>();
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      Matrix3d A;
      // clang-format off
      A << 1, 2, 3,
          0, 1, -2,
          0, 0, 0;
      // clang-format on
      Vector3d b_lb(0, -std::numeric_limits<double>::infinity(), -1);
      Vector3d b_ub(10, 3, 0);
      prog()->AddLinearConstraint(A, b_lb, b_ub, x_);
      prog()->AddBoundingBoxConstraint(1.0, numeric_limits<double>::infinity(),
                                       x_(1));
      break;
    }
    case ConstraintForm::kSymbolic: {
      Vector2<Expression> expr;
      // clang-format off
      expr << x_(0) + 2 * x_(1) + 3 * x_(2),
          x_(1) - 2 * x_(2);
      // clang-format on
      prog()->AddLinearConstraint(
          expr, Eigen::Vector2d(0, -numeric_limits<double>::infinity()),
          Vector2d(10, 3));
      prog()->AddBoundingBoxConstraint(1, numeric_limits<double>::infinity(),
                                       x_(1));
      break;
    }
    case ConstraintForm::kFormula: {
      prog()->AddLinearConstraint(x_(0) + 2 * x_(1) + 3 * x_(2), 0, 10);
      prog()->AddLinearConstraint(x_(1) - 2 * x_(2) <= 3);
      prog()->AddLinearConstraint(+x_(1) >= 1);
      break;
    }
    default: { throw std::runtime_error("Unknown constraint form"); }
  }
}

void LinearFeasibilityProgram::CheckSolution(
    const MathematicalProgramResult& result) const {
  auto x_val = result.GetSolution(x_);
  Vector3d A_times_x(x_val(0) + 2 * x_val(1) + 3 * x_val(2),
                     x_val(1) - 2 * x_val(2), 0);
  EXPECT_GE(A_times_x(0), 0 - 1e-10);
  EXPECT_LE(A_times_x(0), 10 + 1e-10);
  EXPECT_LE(A_times_x(1), 3 + 1E-10);
  EXPECT_LE(A_times_x(2), 0 + 1E-10);
  EXPECT_GE(A_times_x(2), 0 - 1E-10);
  EXPECT_GE(result.GetSolution(x_(1)), 1 - 1E-10);
}

LinearProgram0::LinearProgram0(CostForm cost_form,
                               ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form), x_(), x_expected_(1, 2) {
  x_ = prog()->NewContinuousVariables<2>();
  switch (cost_form) {
    case CostForm::kNonSymbolic: {
      prog()->AddLinearCost(Vector2d(2.0, 1.0), 4, x_);
      break;
    }
    case CostForm::kSymbolic: {
      prog()->AddLinearCost(2 * x_(0) + x_(1) + 4);
      break;
    }
    default: { throw std::runtime_error("Un-supported cost form."); }
  }
  Vector3d b_lb(-numeric_limits<double>::infinity(), 2.0,
                -numeric_limits<double>::infinity());
  Vector3d b_ub(1.0, numeric_limits<double>::infinity(), 4.0);
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      Eigen::Matrix<double, 3, 2> A;
      // clang-format off
      A << -1, 1,
          1, 1,
          1, -2;
      // clang-format on
      prog()->AddLinearConstraint(A, b_lb, b_ub, x_);
      prog()->AddBoundingBoxConstraint(
          Vector2d(0, 2),
          Vector2d::Constant(numeric_limits<double>::infinity()), x_);
      break;
    }
    case ConstraintForm::kSymbolic: {
      Vector3<Expression> expr1;
      // clang-format off
      expr1 << -x_(0) + x_(1),
          x_(0) + x_(1),
          x_(0) - 2 * x_(1);
      // clang-format on
      prog()->AddLinearConstraint(expr1, b_lb, b_ub);
      prog()->AddBoundingBoxConstraint(
          Vector2d(0, 2),
          Vector2d::Constant(numeric_limits<double>::infinity()), x_);
      break;
    }
    case ConstraintForm::kFormula: {
      prog()->AddLinearConstraint(-x_(0) + x_(1) <= 1);
      prog()->AddLinearConstraint(x_(0) + x_(1) >= 2);
      prog()->AddLinearConstraint(x_(0) - 2 * x_(1) <= 4);
      prog()->AddLinearConstraint(+x_(1) >= 2);
      prog()->AddLinearConstraint(+x_(0) >= 0);
      break;
    }
    default: { throw std::runtime_error("Unsupported constraint form."); }
  }
}

void LinearProgram0::CheckSolution(
    const MathematicalProgramResult& result) const {
  const double tol = GetSolverSolutionDefaultCompareTolerance(
      result.get_solver_id());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

LinearProgram1::LinearProgram1(CostForm cost_form,
                               ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form), x_{}, x_expected_(0, 4) {
  x_ = prog()->NewContinuousVariables<2>();
  switch (cost_form) {
    case CostForm::kNonSymbolic: {
      prog()->AddLinearCost(Vector2d(1.0, -2.0), 3, x_);
      break;
    }
    case CostForm::kSymbolic: {
      prog()->AddLinearCost(x_(0) - 2 * x_(1) + 3);
      break;
    }
    default:
      throw std::runtime_error("Unsupported cost form.");
  }
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic:
    case ConstraintForm::kSymbolic: {
      prog()->AddBoundingBoxConstraint(Vector2d(0, -1), Vector2d(2, 4), x_);
      break;
    }
    case ConstraintForm::kFormula: {
      prog()->AddLinearConstraint(+x_(0) >= 0);
      prog()->AddLinearConstraint(+x_(0) <= 2);
      prog()->AddLinearConstraint(+x_(1) >= -1);
      prog()->AddLinearConstraint(+x_(1) <= 4);
      break;
    }
    default:
      throw std::runtime_error("Unsupported constraint form.");
  }
}

void LinearProgram1::CheckSolution(
    const MathematicalProgramResult& result) const {
  const double tol = GetSolverSolutionDefaultCompareTolerance(
      result.get_solver_id());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

LinearProgram2::LinearProgram2(CostForm cost_form,
                               ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form),
      x_(),
      x_expected_(0, 0, 15, 25.0 / 3.0) {
  x_ = prog()->NewContinuousVariables<4>();
  switch (cost_form) {
    case CostForm::kNonSymbolic: {
      prog()->AddLinearCost(Vector3d(-3, -1, -4), x_.head<3>());
      prog()->AddLinearCost(Vector2d(-1, -1), x_.tail<2>());
      break;
    }
    case CostForm::kSymbolic: {
      prog()->AddLinearCost(-3 * x_(0) - x_(1) - 4 * x_(2));
      prog()->AddLinearCost(-x_(2) - x_(3));
      break;
    }
    default:
      throw std::runtime_error("Unsupported cost term.");
  }

  Vector4d b_lb(15, -numeric_limits<double>::infinity(),
                -numeric_limits<double>::infinity(), -100);
  Vector4d b_ub(numeric_limits<double>::infinity(), 25,
                numeric_limits<double>::infinity(), 40);
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      prog()->AddLinearEqualityConstraint(Eigen::RowVector3d(3, 1, 2), 30,
                                          x_.head<3>());

      Matrix4d A;
      // clang-format off
      A << 2, 1, 3, 1,
          0, 2, 0, 3,
          1, 2, 0, 1,
          1, 0, 0, 2;
      // clang-format on

      prog()->AddLinearConstraint(A, b_lb, b_ub, x_);
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      prog()->AddBoundingBoxConstraint(0, 10, x_.segment<1>(1));
      break;
    }
    case ConstraintForm::kSymbolic: {
      prog()->AddLinearEqualityConstraint(3 * x_(0) + x_(1) + 2 * x_(2), 30);
      Eigen::Matrix<Expression, 4, 1> expr;
      // clang-format off
      expr << 2 * x_(0) + x_(1) + 3 * x_(2) + x_(3),
          2 * x_(1) + 3 * x_(3),
          x_(0) + 2 * x_(1) + x_(3),
          x_(0) + 2 * x_(2);
      // clang-format on
      prog()->AddLinearConstraint(expr, b_lb, b_ub);
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      prog()->AddBoundingBoxConstraint(0, 10, x_.segment<1>(1));
      break;
    }
    case ConstraintForm::kFormula: {
      prog()->AddLinearConstraint(3 * x_(0) + x_(1) + 2 * x_(2) == 30);
      prog()->AddLinearConstraint(2 * x_(0) + x_(1) + 3 * x_(2) + x_(3) >= 15);
      prog()->AddLinearConstraint(2 * x_(1) + 3 * x_(3) <= 25);
      // TODO(hongkai.dai) : uncomment the next line when the bug expression >=
      // -inf is fixed.
      // prog()->AddLinearConstraint(x_(0) + 2 * x_(1) + x_(3) >=
      // -numeric_limits<double>::infinity());
      prog()->AddLinearConstraint(x_(0) + 2 * x_(2) <= 40);
      prog()->AddLinearConstraint(x_(0) + 2 * x_(2) >= -100);
      prog()->AddLinearConstraint(+x_(0) >= 0);
      prog()->AddLinearConstraint(+x_(1) >= 0);
      prog()->AddLinearConstraint(+x_(2) >= 0);
      prog()->AddLinearConstraint(+x_(3) >= 0);
      prog()->AddLinearConstraint(+x_(1) <= 10);
      break;
    }
    default:
      throw std::runtime_error("Unsupported constraint form.");
  }
}

void LinearProgram2::CheckSolution(
    const MathematicalProgramResult& result) const {
  const double tol = GetSolverSolutionDefaultCompareTolerance(
      result.get_solver_id());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, tol);
}

LinearProgram3::LinearProgram3(CostForm cost_form,
                               ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form),
      x_(),
      x_expected_(8, 3, 11) {
  x_ = prog()->NewContinuousVariables<3>("x");
  switch (cost_form) {
    case CostForm::kNonSymbolic: {
      prog()->AddLinearCost(Eigen::Vector3d(4, 5, 6), x_);
      break;
    }

    case CostForm::kSymbolic: {
      prog()->AddLinearCost(4 * x_(0) + 5 * x_(1) + 6 * x_(2));
      break;
    }
    default:
      throw std::runtime_error("Unsupported cost form.");
  }
  Eigen::Vector3d b_lb(11, -numeric_limits<double>::infinity(), 35);
  Eigen::Vector3d b_ub(numeric_limits<double>::infinity(), 5,
                       numeric_limits<double>::infinity());
  switch (constraint_form) {
    case ConstraintForm::kNonSymbolic: {
      prog()->AddLinearEqualityConstraint(Eigen::RowVector3d(1, -1, -1), 0,
                                          {x_.segment<1>(2), x_.head<2>()});
      Eigen::Matrix<double, 3, 2> A;
      // clang-format off
      A << 1, 1,
          1, -1,
          7, 12;
      // clang-format on
      prog()->AddLinearConstraint(A, b_lb, b_ub, x_.head<2>());
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      break;
    }
    case ConstraintForm::kSymbolic: {
      prog()->AddLinearEqualityConstraint(x_(2) - x_(0) - x_(1), 0);
      Vector3<Expression> expr;
      // clang-format off
      expr << x_(0) + x_(1),
          x_(0) - x_(1),
          7 * x_(0) + 12 * x_(1);
      // clang-format on
      prog()->AddLinearConstraint(expr, b_lb, b_ub);
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(),
                                       x_);
      break;
    }
    case ConstraintForm::kFormula: {
      prog()->AddLinearConstraint(x_(2) - x_(0) - x_(1) == 0);
      prog()->AddLinearConstraint(x_(0) + x_(1) >= 11);
      prog()->AddLinearConstraint(x_(0) - x_(1) <= 5);
      prog()->AddLinearConstraint(7 * x_(0) >= 35 - 12 * x_(1));
      prog()->AddLinearConstraint(+x_(0) >= 0);
      prog()->AddLinearConstraint(+x_(1) >= 0);
      prog()->AddLinearConstraint(+x_(2) >= 0);
      break;
    }
    default:
      throw std::runtime_error("Unsupported constraint form.");
  }
}

void LinearProgram3::CheckSolution(
    const MathematicalProgramResult& result) const {
  // Mosek has a looser tolerance.
  double tol = GetSolverSolutionDefaultCompareTolerance(result.get_solver_id());
  if (result.get_solver_id() == MosekSolver::id()) {
    tol = 1E-6;
  }
  // Ipopt has a looser objective tolerance
  double cost_tol = tol;
  if (result.get_solver_id() == IpoptSolver::id()) {
    cost_tol = 1E-5;
  }
  EXPECT_TRUE(CompareMatrices(result.GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), result, cost_tol);
}

std::ostream& operator<<(std::ostream& os, LinearProblems value) {
  os << "LinearProblems::";
  switch (value) {
    case LinearProblems::kLinearFeasibilityProgram: {
      os << "kLinearFeasibilityProgram";
      return os;
    }
    case LinearProblems::kLinearProgram0: {
      os << "kLinearFeasibilityProgram0";
      return os;
    }
    case LinearProblems::kLinearProgram1: {
      os << "kLinearFeasibilityProgram1";
      return os;
    }
    case LinearProblems::kLinearProgram2: {
      os << "kLinearFeasibilityProgram2";
      return os;
    }
    case LinearProblems::kLinearProgram3: {
      os << "kLinearFeasibilityProgram3";
      return os;
    }
  }
  DRAKE_UNREACHABLE();
}

LinearProgramTest::LinearProgramTest() {
  auto cost_form = std::get<0>(GetParam());
  auto constraint_form = std::get<1>(GetParam());
  switch (std::get<2>(GetParam())) {
    case LinearProblems::kLinearFeasibilityProgram: {
      prob_ = std::make_unique<LinearFeasibilityProgram>(constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram0: {
      prob_ = std::make_unique<LinearProgram0>(cost_form, constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram1: {
      prob_ = std::make_unique<LinearProgram1>(cost_form, constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram2: {
      prob_ = std::make_unique<LinearProgram2>(cost_form, constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram3: {
      prob_ = std::make_unique<LinearProgram3>(cost_form, constraint_form);
      break;
    }
    default:
      throw std::runtime_error("Un-recognized linear problem.");
  }
}

std::vector<LinearProblems> linear_problems() {
  return std::vector<LinearProblems>{
      LinearProblems::kLinearFeasibilityProgram,
      LinearProblems::kLinearProgram0, LinearProblems::kLinearProgram1,
      LinearProblems::kLinearProgram2, LinearProblems::kLinearProgram3};
}

InfeasibleLinearProgramTest0::InfeasibleLinearProgramTest0()
    : prog_(std::make_unique<MathematicalProgram>()) {
  auto x = prog_->NewContinuousVariables<2>("x");
  prog_->AddLinearCost(-x(0) - x(1));
  prog_->AddLinearConstraint(x(0) + 2 * x(1) <= 3);
  prog_->AddLinearConstraint(2 * x(0) + x(1) == 4);
  prog_->AddLinearConstraint(x(0) >= 0);
  prog_->AddLinearConstraint(x(1) >= 2);
}

UnboundedLinearProgramTest0::UnboundedLinearProgramTest0()
    : prog_(std::make_unique<MathematicalProgram>()) {
  x_ = prog_->NewContinuousVariables<2>("x");
  prog_->AddLinearCost(-x_(0) - x_(1));
  prog_->AddLinearConstraint(2 * x_(0) + x_(1) >= 4);
  prog_->AddLinearConstraint(x_(0) >= 0);
  prog_->AddLinearConstraint(x_(1) >= 2);
}

UnboundedLinearProgramTest1::UnboundedLinearProgramTest1()
    : prog_(std::make_unique<MathematicalProgram>()) {
  auto x = prog_->NewContinuousVariables<4>("x");
  prog_->AddCost(x(0) + 2 * x(1) + 3 * x(2) + 2.5 * x(3) + 2);
  prog_->AddLinearConstraint(x(0) + x(1) - x(2) + x(3) <= 3);
  prog_->AddLinearConstraint(x(0) + 2 * x(1) - 2 * x(2) + 4 * x(3), 1, 3);
  prog_->AddBoundingBoxConstraint(0, 1, VectorDecisionVariable<2>(x(0), x(2)));
}

DuplicatedVariableLinearProgramTest1::DuplicatedVariableLinearProgramTest1()
    : prog_{std::make_unique<MathematicalProgram>()} {
  x_ = prog_->NewContinuousVariables<3>();
  // Intentionally add the cost with duplicated entries x_(1).
  prog_->AddLinearCost(Eigen::Vector4d(1, 2, 1, 4),
                       Vector4<symbolic::Variable>(x_(0), x_(1), x_(1), x_(2)));
  prog_->AddLinearCost(Eigen::Vector2d(2, -2), 3,
                       Vector2<symbolic::Variable>(x_(0), x_(0)));
  prog_->AddLinearEqualityConstraint(
      Eigen::RowVector3d(2, 2, -1), Vector1d(1),
      Vector3<symbolic::Variable>(x_(0), x_(2), x_(0)));
  Eigen::Matrix<double, 2, 5> A;
  // A * vars = [x0 + 2*x1 + x2]
  //            [2*x0 + x1     ]
  // clang-format off
  A << -1, 1, 3, 2, -1,
       2, 2, -1, 1, -1;
  // clang-format on
  Eigen::Matrix<symbolic::Variable, 5, 1> vars;
  vars << x_(2), x_(0), x_(2), x_(1), x_(2);
  prog_->AddLinearConstraint(A, Eigen::Vector2d(-kInf, 1),
                             Eigen::Vector2d(3, 3), vars);
  prog_->AddLinearConstraint(
      Eigen::RowVector4d(2, 3, 2, -1), Vector1d(0), Vector1d(kInf),
      Vector4<symbolic::Variable>(x_(1), x_(2), x_(0), x_(0)));
}

void DuplicatedVariableLinearProgramTest1::CheckSolution(
    const SolverInterface& solver,
    const std::optional<SolverOptions>& solver_options, double tol) const {
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(*prog_, std::nullopt, solver_options, &result);
    EXPECT_TRUE(result.is_success());
    const Eigen::Vector3d x_sol = result.GetSolution(x_);
    EXPECT_NEAR(result.get_optimal_cost(),
                x_sol(0) + 3 * x_sol(1) + 4 * x_sol(2) + 3, tol);
    EXPECT_LE(x_sol(0) + 2 * x_sol(1) + x_sol(2), 3 + tol);
    EXPECT_NEAR(x_sol(0) + 2 * x_sol(2), 1, tol);
    EXPECT_GE(2 * x_sol(0) + x_sol(1), 1 - tol);
    EXPECT_LE(2 * x_sol(0) + x_sol(1), 3 + tol);
    EXPECT_GE(x_sol(0) + 2 * x_sol(1) + 3 * x_sol(2), -tol);
  }
}

void TestLPDualSolution1(const SolverInterface& solver, double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  Eigen::Matrix<double, 5, 2> A;
  // clang-format off
  A << 1, 2,
       3, 4,
       2, 1,
       1, 5,
       3, 2;
  // clang-format on
  Eigen::Matrix<double, 5, 1> lb, ub;
  lb << -kInf, -2, -3, -kInf, 4;
  ub << 3, kInf, 4, kInf, 5;
  auto constraint = prog.AddLinearConstraint(A, lb, ub, x);
  auto cost = prog.AddLinearCost(2 * x[0] + 3 * x[1]);
  if (solver.available()) {
    MathematicalProgramResult result1;
    solver.Solve(prog, {}, {}, &result1);
    EXPECT_TRUE(result1.is_success());
    Eigen::Matrix<double, 5, 1> dual_solution_expected;
    // This dual solution is computed by first finding the active constraint at
    // the optimal solution, denoted as Aeq * x = beq, then the dual solution
    // equals to cᵀAeq⁻¹.
    dual_solution_expected << 0, 0.8, -0.2, 0, 0;
    EXPECT_TRUE(CompareMatrices(result1.GetDualSolution(constraint),
                                dual_solution_expected, tol));

    cost.evaluator()->UpdateCoefficients(Eigen::Vector2d(-3, -4));
    MathematicalProgramResult result2;
    solver.Solve(prog, {}, {}, &result2);
    EXPECT_TRUE(result2.is_success());
    Eigen::Matrix2d Aeq;
    Aeq.row(0) = A.row(0);
    Aeq.row(1) = A.row(4);
    dual_solution_expected << -1.5, 0, 0, 0, -0.5;
    EXPECT_TRUE(CompareMatrices(result2.GetDualSolution(constraint),
                                dual_solution_expected, tol));
  }
}

void TestLPDualSolution2(const SolverInterface& solver, double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto constraint = prog.AddBoundingBoxConstraint(Eigen::Vector2d(-1, 2),
                                                  Eigen::Vector2d(3, 5), x);
  prog.AddLinearCost(x[0] - x[1]);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint),
                                Eigen::Vector2d(1, -1), tol));
  }
}

void TestLPDualSolution2Scaled(const SolverInterface& solver, double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto constraint = prog.AddBoundingBoxConstraint(Eigen::Vector2d(-1, 2),
                                                  Eigen::Vector2d(3, 5), x);
  prog.AddLinearCost(x[0] - x[1]);
  prog.SetVariableScaling(x[0], 10);
  prog.SetVariableScaling(x[1], 0.1);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint),
                                Eigen::Vector2d(1, -1), tol));
  }
}

void TestLPDualSolution3(const SolverInterface& solver, double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto constraint1 = prog.AddBoundingBoxConstraint(-2, 1, x);
  auto constraint2 = prog.AddBoundingBoxConstraint(-1, 2, x[1]);
  auto constraint3 = prog.AddBoundingBoxConstraint(-1, 3, x[0]);
  prog.AddLinearCost(-x[0] + x[1]);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, {}, {}, &result);
    EXPECT_TRUE(result.is_success());
    // The optimal solution is (1, -1).
    // constraint1 has an upper bound x(0) <= 1 being active.
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint1),
                                Eigen::Vector2d(-1, 0), tol));
    // constraint2 has a lower bound x(1) >= -1 being active.
    EXPECT_TRUE(
        CompareMatrices(result.GetDualSolution(constraint2), Vector1d(1), tol));
    // constraint 3 is not active.
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint3),
                                Vector1d::Zero(), tol));
  }
}

void TestLPPoorScaling1(const SolverInterface& solver, bool expect_success,
                        double tol,
                        const std::optional<SolverOptions>& options) {
  MathematicalProgram prog;
  const auto alpha = prog.NewContinuousVariables<4>();
  const auto z = prog.NewContinuousVariables<1>()(0);
  const double eps = 5.5511151231257827e-17;
  Eigen::Matrix<double, 2, 4> vertices;
  // clang-format off
  vertices << eps, 1, eps, 1,
              eps, 1, 2, 2;
  // clang-format on
  const Eigen::Vector2d pt(0.99, 1.99);
  prog.AddLinearConstraint(-z + vertices.row(0).dot(alpha) <= pt(0));
  prog.AddLinearConstraint(-z + vertices.row(1).dot(alpha) <= pt(1));
  prog.AddLinearConstraint(z + vertices.row(0).dot(alpha) >= pt(0));
  prog.AddLinearConstraint(z + vertices.row(1).dot(alpha) >= pt(1));
  prog.AddBoundingBoxConstraint(0, 1, alpha);
  prog.AddLinearCost(z);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, std::nullopt, options, &result);
    const auto alpha_sol = result.GetSolution(alpha);
    ASSERT_TRUE(result.is_success());
    if (expect_success) {
      EXPECT_TRUE(CompareMatrices(vertices * alpha_sol, pt, tol));
      EXPECT_NEAR(result.GetSolution(z), 0, tol);
    } else {
      EXPECT_GT(result.GetSolution(z), tol);
    }
  }
}

void TestLPPoorScaling2(const SolverInterface& solver, bool expect_success,
                        double tol,
                        const std::optional<SolverOptions>& options) {
  Eigen::Matrix<double, 10, 3> A;
  A.topRows<3>() = Eigen::Matrix3d::Identity();
  A.middleRows<3>(3) = -Eigen::Matrix3d::Identity();
  // clang-format off
  A.bottomRows<4>() << 9.99400723e-01, -1.75186298e-21,  3.46149414e-02,
  9.82328785e-01,  1.82238474e-20, -1.87163452e-01,
  9.68599420e-01, -2.48626556e-01, -1.27003703e-12,
  9.68599420e-01,  2.48626556e-01, -1.27003703e-12;
  // clang-format on
  Eigen::Matrix<double, 10, 1> b;
  b << 1.0, 0.5, 1.0, -0.0, 0.5, -0.0, 0.46783912, 0.39023174, 0.50647967,
      0.50647967;

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto r = prog.NewContinuousVariables<1>()(0);
  for (int i = 0; i < A.rows(); ++i) {
    prog.AddLinearConstraint(A.row(i).dot(x) + A.row(i).norm() * r <= b(i));
  }
  prog.AddBoundingBoxConstraint(0, kInf, r);
  prog.AddLinearCost(-r);
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(prog, std::nullopt, options, &result);
    ASSERT_TRUE(result.is_success());
    const Eigen::Vector3d x_expected(0.2282, 0, 0.3323);
    if (expect_success) {
      EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, tol));
    } else {
      EXPECT_GT((result.GetSolution(x) - x_expected).norm(), tol);
    }
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
