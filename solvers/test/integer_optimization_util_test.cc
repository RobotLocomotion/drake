#include "drake/solvers/integer_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"

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
      cost.constraint()->UpdateCoefficients(Vector1d(1));
      b0_cnstr_.constraint()->UpdateLowerBound(Vector1d(b_vals_[i](0)));
      b0_cnstr_.constraint()->UpdateUpperBound(Vector1d(b_vals_[i](0)));
      b1_cnstr_.constraint()->UpdateLowerBound(Vector1d(b_vals_[i](1)));
      b1_cnstr_.constraint()->UpdateUpperBound(Vector1d(b_vals_[i](1)));
      auto result = prog_.Solve();
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      EXPECT_NEAR(prog_.GetSolution(operand_result), operand_result_expected,
                  1E-3);

      // Now update the objective to
      // min -b_and
      // The solution should be the same.
      cost.constraint()->UpdateCoefficients(Vector1d(-1));
      result = prog_.Solve();
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      EXPECT_NEAR(prog_.GetSolution(operand_result), operand_result_expected,
                  1E-3);
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

TEST_F(IntegerOptimizationUtilTest, TesXor) {
  CheckLogicalOperand(LogicalOperand::kXor);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
