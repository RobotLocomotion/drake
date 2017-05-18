#include "drake/solvers/test/linear_program_examples.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
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

void LinearFeasibilityProgram::CheckSolution(SolverType) const {
  auto x_val = prog()->GetSolution(x_);
  Vector3d A_times_x(x_val(0) + 2 * x_val(1) + 3 * x_val(2),
                     x_val(1) - 2 * x_val(2), 0);
  EXPECT_GE(A_times_x(0), 0 - 1e-10);
  EXPECT_LE(A_times_x(0), 10 + 1e-10);
  EXPECT_LE(A_times_x(1), 3 + 1E-10);
  EXPECT_LE(A_times_x(2), 0 + 1E-10);
  EXPECT_GE(A_times_x(2), 0 - 1E-10);
  EXPECT_GE(prog()->GetSolution(x_(1)), 1 - 1E-10);
}

LinearProgram0::LinearProgram0(CostForm cost_form,
                               ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form), x_(), x_expected_(1, 2) {
  x_ = prog()->NewContinuousVariables<2>();
  switch (cost_form) {
    case CostForm::kNonSymbolic: {
      prog()->AddLinearCost(Vector2d(2.0, 1.0), x_);
      break;
    }
    case CostForm::kSymbolic: {
      prog()->AddLinearCost(2 * x_(0) + x_(1));
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

void LinearProgram0::CheckSolution(SolverType solver_type) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(solver_type);
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), tol);
}

LinearProgram1::LinearProgram1(CostForm cost_form,
                               ConstraintForm constraint_form)
    : OptimizationProgram(cost_form, constraint_form), x_{}, x_expected_(0, 4) {
  x_ = prog()->NewContinuousVariables<2>();
  switch (cost_form) {
    case CostForm::kNonSymbolic: {
      prog()->AddLinearCost(Vector2d(1.0, -2.0), x_);
      break;
    }
    case CostForm::kSymbolic: {
      prog()->AddLinearCost(x_(0) - 2 * x_(1));
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

void LinearProgram1::CheckSolution(SolverType solver_type) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(solver_type);
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), tol);
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

void LinearProgram2::CheckSolution(SolverType solver_type) const {
  double tol = GetSolverSolutionDefaultCompareTolerance(solver_type);
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), tol);
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

void LinearProgram3::CheckSolution(SolverType solver_type) const {
  // Mosek has a looser tolerance.
  double tol = GetSolverSolutionDefaultCompareTolerance(solver_type);
  if (solver_type == SolverType::kMosek) {
    tol = 1E-6;
  }
  // Ipopt has a looser objective tolerance
  double cost_tol = tol;
  if (solver_type == SolverType::kIpopt) {
    cost_tol = 1E-5;
  }
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, tol,
                              MatrixCompareType::absolute));
  ExpectSolutionCostAccurate(*prog(), cost_tol);
}


LinearProgramTest::LinearProgramTest() {
  auto cost_form = std::get<0>(GetParam());
  auto constraint_form = std::get<1>(GetParam());
  switch (std::get<2>(GetParam())) {
    case LinearProblems::kLinearFeasibilityProgram : {
      prob_ = std::make_unique<LinearFeasibilityProgram>(constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram0 : {
      prob_ = std::make_unique<LinearProgram0>(cost_form, constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram1 : {
      prob_ = std::make_unique<LinearProgram1>(cost_form, constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram2 : {
      prob_ = std::make_unique<LinearProgram2>(cost_form, constraint_form);
      break;
    }
    case LinearProblems::kLinearProgram3 : {
      prob_ = std::make_unique<LinearProgram3>(cost_form, constraint_form);
      break;
    }
    default : throw std::runtime_error("Un-recognized linear problem.");
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
  auto x = prog_->NewContinuousVariables<2>("x");
  prog_->AddLinearCost(-x(0) - x(1));
  prog_->AddLinearConstraint(2 * x(0) + x(1) >= 4);
  prog_->AddLinearConstraint(x(0) >= 0);
  prog_->AddLinearConstraint(x(1) >= 2);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
