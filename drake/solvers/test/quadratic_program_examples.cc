#include "drake/solvers/test/quadratic_program_examples.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
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

namespace drake {
namespace solvers {
namespace test {

QuadraticProgramTest::QuadraticProgramTest() {
  auto cost_form = std::get<0>(GetParam());
  auto cnstr_form = std::get<1>(GetParam());
  switch (std::get<2>(GetParam())) {
    case QuadraticProblems::kQuadraticProgram0 : {
      prob_ = std::make_unique<QuadraticProgram0>(cost_form, cnstr_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram1 : {
      prob_ = std::make_unique<QuadraticProgram1>(cost_form, cnstr_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram2 : {
      prob_ = std::make_unique<QuadraticProgram2>(cost_form, cnstr_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram3 : {
      prob_ = std::make_unique<QuadraticProgram3>(cost_form, cnstr_form);
      break;
    }
    case QuadraticProblems::kQuadraticProgram4 : {
      prob_ = std::make_unique<QuadraticProgram4>(cost_form, cnstr_form);
      break;
    }
    default : throw std::runtime_error("Un-recognized quadratic problem.");
  }
}

std::vector<QuadraticProblems> quadratic_problems() {
  return std::vector<QuadraticProblems>{
      QuadraticProblems::kQuadraticProgram0, QuadraticProblems::kQuadraticProgram1,
  QuadraticProblems::kQuadraticProgram2, QuadraticProblems::kQuadraticProgram3,
      QuadraticProblems::kQuadraticProgram4};
}

QuadraticProgram0::QuadraticProgram0(CostForm cost_form, ConstraintForm cnstr_form)
    : OptimizationProgram(cost_form, cnstr_form), x_(), x_expected_(0.25, 0.75) {
  x_ = prog()->NewContinuousVariables<2>();
  switch (cost_form) {
    case CostForm::kNonSymbolic : {
      Matrix2d Q;
      // clang-format off
      Q << 4, 2,
          0, 2;
      // clang-format on
      Vector2d b(1, 0);
      prog()->AddQuadraticCost(Q, b, x_);
      prog()->AddLinearCost(Vector1d(1.0), x_.segment<1>(1));
      break;
    }
    default: {
      throw std::runtime_error("Unsupported cost form.");
    }
  }
  switch (cnstr_form) {
    case ConstraintForm::kNonSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(), x_);
      prog()->AddBoundingBoxConstraint(-1, numeric_limits<double>::infinity(), x_(1));
      prog()->AddLinearEqualityConstraint(RowVector2d(1, 1), 1, x_);
      break;
    }
    case ConstraintForm::kSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(), x_);
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

void QuadraticProgram0::CheckSolution() const {
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, 1E-8, MatrixCompareType::absolute));
}

QuadraticProgram1::QuadraticProgram1(CostForm cost_form,
                                     ConstraintForm cnstr_form)
    : OptimizationProgram(cost_form, cnstr_form), x_{}, x_expected_(0, 1, 2.0 / 3){
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
    default : throw std::runtime_error("Unsupported cost form.");
  }
  Vector4d b_lb(4, -numeric_limits<double>::infinity(), -20,
                -numeric_limits<double>::infinity());
  Vector4d b_ub(numeric_limits<double>::infinity(), -1, 100,
                numeric_limits<double>::infinity());
  switch (cnstr_form) {
    case ConstraintForm::kNonSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(), x_);
      Eigen::Matrix<double, 4, 3> A1;
      A1 << 1, 2, 3, -1, -1, 0, 0, 1, 2, 1, 1, 2;

      prog()->AddLinearConstraint(A1, b_lb, b_ub, x_);
      // This test also handles linear equality constraint
      prog()->AddLinearEqualityConstraint(Eigen::RowVector3d(3, 1, 3), 3, x_);
      break;
    }
    case ConstraintForm::kSymbolic : {
      prog()->AddBoundingBoxConstraint(0, numeric_limits<double>::infinity(), x_);
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
      // TODO(hongkai.dai): Uncoment the next line, when we resolves the error with
      // infinity on the right handside of a formula.
      // prog()->AddLinearConstraint(x_(0) + x_(1) + 2 * x_(2) <= numeric_limits<double>::infinity());
      prog()->AddLinearConstraint(3 * x_(0) + x_(1) + 3 * x_(2) == 3);
      break;
    }
    default : throw std::runtime_error("Unsupported constraint form.");
  }
}

void QuadraticProgram1::CheckSolution() const {
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, 1E-8, MatrixCompareType::absolute));
}

QuadraticProgram2::QuadraticProgram2(CostForm cost_form, ConstraintForm cnstr_form)
    : OptimizationProgram(cost_form, cnstr_form), x_{}, x_expected_{} {
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
    default : throw std::runtime_error("Unsupported cost form.");
  }

  Eigen::Matrix<double, 5, 5> Q_symmetric = 0.5 * (Q + Q.transpose());
  x_expected_ = -Q_symmetric.llt().solve(b);
}

void QuadraticProgram2::CheckSolution() const {
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, 1E-8, MatrixCompareType::absolute));
}

QuadraticProgram3::QuadraticProgram3(CostForm cost_form,
                                     ConstraintForm cnstr_form)
: OptimizationProgram(cost_form, cnstr_form), x_{}, x_expected_{} {
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
    default : throw std::runtime_error("Unsupported cost form.");
  }

  Eigen::Matrix<double, 6, 6> Q;
  Q.setZero();
  Q.topLeftCorner<4, 4>() = Q1;
  Q.bottomRightCorner<4, 4>() += Q2;
  Eigen::Matrix<double, 6, 6> Q_symmetric = 0.5 * (Q + Q.transpose());

  Eigen::Matrix<double, 6, 1> b;
  b.setZero();
  b.head<4>() = b1;
  b.tail<4>() += b2;

  x_expected_ = -Q_symmetric.llt().solve(b);
}

void QuadraticProgram3::CheckSolution() const {
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, 1E-8, MatrixCompareType::absolute));
}

QuadraticProgram4::QuadraticProgram4(CostForm cost_form,
                                     ConstraintForm cnstr_form)
: OptimizationProgram(cost_form, cnstr_form), x_{}, x_expected_(0.8, 0.2, 0.6) {
  x_ = prog()->NewContinuousVariables<3>();
  switch (cost_form) {
    case CostForm::kNonSymbolic : {
      Matrix3d Q = Matrix3d::Identity();
      Q(2, 2) = 2.0;
      Vector3d b = Vector3d::Zero();
      prog()->AddQuadraticCost(Q, b, x_);
      break;
    }
    default : throw std::runtime_error("Unsupported cost form.");
  }
  switch (cnstr_form) {
    case ConstraintForm::kNonSymbolic : {
      prog()->AddLinearEqualityConstraint(RowVector2d(1, 1), 1, x_.head<2>());
      prog()->AddLinearEqualityConstraint(RowVector2d(1, 2), 2, {x_.segment<1>(0), x_.segment<1>(2)});
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

void QuadraticProgram4::CheckSolution() const {
  EXPECT_TRUE(CompareMatrices(prog()->GetSolution(x_), x_expected_, 1E-9, MatrixCompareType::absolute));
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
