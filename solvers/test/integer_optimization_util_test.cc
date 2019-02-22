#include "drake/solvers/integer_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace {
class IntegerOptimizationUtilTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntegerOptimizationUtilTest)

  enum class LogicalOperand { kAnd, kOr, kXor };

  IntegerOptimizationUtilTest()
      : prog_{},
        b_{prog_.NewContinuousVariables<2>()},
        b_vals_{{Eigen::Vector2i(0, 0), Eigen::Vector2i(0, 1),
                 Eigen::Vector2i(1, 0), Eigen::Vector2i(1, 1)}},
        b0_cnstr_{prog_.AddLinearConstraint(b_(0) == 0)},
        b1_cnstr_{prog_.AddLinearConstraint(b_(1) == 0)} {}

  void CheckLogicalOperand(LogicalOperand operand) {
    // We want to make sure that given that b0 and b1 are fixed, the only
    // solution to satisfying the constraints, is that
    // operand_result = b0 operand b1.
    symbolic::Variable operand_result = prog_.NewContinuousVariables<1>()(0);
    switch (operand) {
      case LogicalOperand::kAnd:
        prog_.AddConstraint(CreateLogicalAndConstraint(
            symbolic::Expression{b_(0)}, symbolic::Expression{b_(1)},
            symbolic::Expression{operand_result}));
        break;
      case LogicalOperand::kOr:
        prog_.AddConstraint(CreateLogicalOrConstraint(
            symbolic::Expression{b_(0)}, symbolic::Expression{b_(1)},
            symbolic::Expression{operand_result}));
        break;
      case LogicalOperand::kXor:
        prog_.AddConstraint(CreateLogicalXorConstraint(
            symbolic::Expression{b_(0)}, symbolic::Expression{b_(1)},
            symbolic::Expression{operand_result}));
        break;
    }
    auto cost = prog_.AddLinearCost(operand_result);
    // Solve the program
    // min operand_result
    // s.t operand_result = b0 operand b1
    //     b0 = b_vals_[i](0)
    //     b1 = b_vals_[i](1).
    for (int i = 0; i < 4; ++i) {
      double operand_result_expected;
      switch (operand) {
        case LogicalOperand::kAnd:
          operand_result_expected =
              static_cast<double>(b_vals_[i](0) & b_vals_[i](1));
          break;
        case LogicalOperand::kOr:
          operand_result_expected =
              static_cast<double>(b_vals_[i](0) | b_vals_[i](1));
          break;
        case LogicalOperand::kXor:
          operand_result_expected =
              static_cast<double>(b_vals_[i](0) ^ b_vals_[i](1));
          break;
      }
      cost.evaluator()->UpdateCoefficients(Vector1d(1));
      b0_cnstr_.evaluator()->UpdateLowerBound(Vector1d(b_vals_[i](0)));
      b0_cnstr_.evaluator()->UpdateUpperBound(Vector1d(b_vals_[i](0)));
      b1_cnstr_.evaluator()->UpdateLowerBound(Vector1d(b_vals_[i](1)));
      b1_cnstr_.evaluator()->UpdateUpperBound(Vector1d(b_vals_[i](1)));
      MathematicalProgramResult result = Solve(prog_);
      EXPECT_TRUE(result.is_success());
      double tol = 1E-3;
      if (result.get_solver_id() == OsqpSolver::id()) {
        // OSQP can solve linear program, but it is likely to fail in polishing,
        // thus the solution is less accurate.
        tol = 3E-3;
      }
      EXPECT_NEAR(result.GetSolution(operand_result), operand_result_expected,
                  tol);

      // Now update the objective to
      // min -b_and
      // The solution should be the same.
      cost.evaluator()->UpdateCoefficients(Vector1d(-1));
      result = Solve(prog_);
      EXPECT_TRUE(result.is_success());
      EXPECT_NEAR(result.GetSolution(operand_result), operand_result_expected,
                  tol);
    }
  }

 protected:
  MathematicalProgram prog_;
  VectorDecisionVariable<2> b_;
  std::array<Eigen::Vector2i, 4> b_vals_;
  Binding<LinearConstraint> b0_cnstr_;
  Binding<LinearConstraint> b1_cnstr_;
};

TEST_F(IntegerOptimizationUtilTest, TestAnd) {
  CheckLogicalOperand(LogicalOperand::kAnd);
}

TEST_F(IntegerOptimizationUtilTest, TestOr) {
  CheckLogicalOperand(LogicalOperand::kOr);
}

TEST_F(IntegerOptimizationUtilTest, TestXor) {
  CheckLogicalOperand(LogicalOperand::kXor);
}

GTEST_TEST(TestBinaryCodeMatchConstraint, Test) {
  MathematicalProgram prog;
  const auto b = prog.NewContinuousVariables<3>();
  auto b_constraint = prog.AddBoundingBoxConstraint(0, 1, b);

  Eigen::Vector3i b_expected{0, 1, 1};

  const auto match = prog.NewContinuousVariables<1>()(0);
  prog.AddConstraint(CreateBinaryCodeMatchConstraint(b, b_expected, match));

  auto match_constraint = prog.AddBoundingBoxConstraint(0, 1, match);

  for (int b0_val : {0, 1}) {
    for (int b1_val : {0, 1}) {
      for (int b2_val : {0, 1}) {
        for (double match_val : {-0.5, 0.0, 0.5, 1.0}) {
          const Eigen::Vector3d b_val(b0_val, b1_val, b2_val);
          b_constraint.evaluator()->UpdateLowerBound(b_val);
          b_constraint.evaluator()->UpdateUpperBound(b_val);
          match_constraint.evaluator()->UpdateUpperBound(Vector1d(match_val));
          match_constraint.evaluator()->UpdateLowerBound(Vector1d(match_val));
          const auto result = Solve(prog);
          if (b0_val == 0 && b1_val == 1 && b2_val == 1) {
            EXPECT_EQ(result.is_success(), match_val == 1.0);
          } else {
            EXPECT_EQ(result.is_success(), match_val == 0.0);
          }
        }
      }
    }
  }
}
}  // namespace
}  // namespace solvers
}  // namespace drake
