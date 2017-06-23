#include "drake/solvers/mixed_integer_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/gray_code.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(TestMixedIntegerUtil, TestCeilLog2) {
  // Check that CeilLog2(j) returns i + 1 for all j = 2ⁱ + 1, 2ⁱ + 2, ... , 2ⁱ⁺¹
  const int kMaxExponent = 15;
  EXPECT_EQ(0, CeilLog2(1));
  for (int i = 0; i < kMaxExponent; ++i) {
    for (int j = (1 << i) + 1; j <= (1 << (i + 1)); ++j) {
      EXPECT_EQ(i + 1, CeilLog2(j));
    }
  }
}

void LogarithmicSOS2Test(int num_lambda) {
  // Solve the program
  // min λᵀ * λ
  // s.t sum λ = 1
  //     λ in sos2
  // We loop over i such that only λ(i) and λ(i+1) can be strictly positive.
  // The optimal cost is λ(i) = λ(i + 1) = 0.5.
  MathematicalProgram prog;
  auto lambda = prog.NewContinuousVariables(num_lambda, "lambda");
  prog.AddCost(lambda.cast<symbolic::Expression>().dot(lambda));
  auto y =
      AddLogarithmicSOS2Constraint(&prog, lambda.cast<symbolic::Expression>());
  int num_binary_vars = y.rows();
  int num_intervals = num_lambda - 1;
  auto y_assignment = prog.AddBoundingBoxConstraint(0, 1, y);

  // We assign the binary variables y with value i, expressed in Gray code.
  const auto gray_codes = math::CalculateReflectedGrayCodes(num_binary_vars);
  Eigen::VectorXd y_val(num_binary_vars);
  for (int i = 0; i < num_intervals; ++i) {
    y_val.setZero();
    for (int j = 0; j < num_binary_vars; ++j) {
      y_val(j) = gray_codes(i, j);
    }
    y_assignment.constraint()->UpdateLowerBound(y_val);
    y_assignment.constraint()->UpdateUpperBound(y_val);

    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      auto result = gurobi_solver.Solve(prog);
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      const auto lambda_val = prog.GetSolution(lambda);
      Eigen::VectorXd lambda_val_expected = Eigen::VectorXd::Zero(num_lambda);
      lambda_val_expected(i) = 0.5;
      lambda_val_expected(i + 1) = 0.5;
      EXPECT_TRUE(CompareMatrices(lambda_val, lambda_val_expected, 1E-5,
                                  MatrixCompareType::absolute));
    }
  }
}

GTEST_TEST(TestLogarithmicSOS2, Test4Lambda) {
  LogarithmicSOS2Test(4);
}

GTEST_TEST(TestLogarithmicSOS2, Test5Lambda) {
  LogarithmicSOS2Test(5);
}

GTEST_TEST(TestLogarithmicSOS2, Test6Lambda) {
  LogarithmicSOS2Test(6);
}

GTEST_TEST(TestLogarithmicSOS2, Test7Lambda) {
  LogarithmicSOS2Test(7);
}

GTEST_TEST(TestLogarithmicSOS2, Test8Lambda) {
  LogarithmicSOS2Test(8);
}

void LogarithmicSOS1Test(int num_lambda,
                         const Eigen::Ref<const Eigen::MatrixXi>& codes) {
  // Check if we impose the constraint
  // λ is in sos1
  // and assign values to the binary variables,
  // whether the corresponding λ(i) is 1.
  MathematicalProgram prog;
  auto lambda = prog.NewContinuousVariables(num_lambda);
  int num_digits = CeilLog2(num_lambda);
  auto y = prog.NewBinaryVariables(num_digits);
  AddLogarithmicSOS1Constraint(&prog, lambda.cast<symbolic::Expression>(), y,
                               codes);
  auto binary_assignment = prog.AddBoundingBoxConstraint(0, 0, y);
  for (int i = 0; i < num_lambda; ++i) {
    Eigen::VectorXd code = codes.row(i).cast<double>().transpose();
    binary_assignment.constraint()->UpdateLowerBound(code);
    binary_assignment.constraint()->UpdateUpperBound(code);
    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      auto result = gurobi_solver.Solve(prog);
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      Eigen::VectorXd lambda_expected(num_lambda);
      lambda_expected.setZero();
      lambda_expected(i) = 1;
      EXPECT_TRUE(CompareMatrices(prog.GetSolution(lambda), lambda_expected,
                                  1E-6, MatrixCompareType::absolute));
    }
  }
}

GTEST_TEST(TestLogarithmicSOS1, Test2Lambda) {
  Eigen::Matrix<int, 2, 1> codes;
  codes << 0, 1;
  LogarithmicSOS1Test(2, codes);
  // Test a different codes
  codes << 1, 0;
  LogarithmicSOS1Test(2, codes);
}

GTEST_TEST(TestLogarithmicSOS1, Test3Lambda) {
  Eigen::Matrix<int, 3, 2> codes;
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 0;
  // clang-format on
  LogarithmicSOS1Test(3, codes);
  // Test a different codes
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 1;
  // clang-format on
  LogarithmicSOS1Test(3, codes);
}

GTEST_TEST(TestLogarithmicSOS1, Test4Lambda) {
  Eigen::Matrix<int, 4, 2> codes;
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 0,
           1, 1;
  // clang-format on
  LogarithmicSOS1Test(4, codes);
  // Test a different codes
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 1,
           1, 0;
  // clang-format on
  LogarithmicSOS1Test(4, codes);
}

GTEST_TEST(TestLogarithmicSOS1, Test5Lambda) {
  auto codes = math::CalculateReflectedGrayCodes<3>();
  LogarithmicSOS1Test(5, codes.topRows<5>());
}

class BilinearProductMcCormickEnvelopeSOS2Test : public ::testing::TestWithParam<int> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BilinearProductMcCormickEnvelopeSOS2Test)

  BilinearProductMcCormickEnvelopeSOS2Test()
      :
 private:
  MathematicalProgram prog_;
  int num_interval_x_;
  int num_interval_y_;
  symbolic::Variable w_;
  symbolic::Variable x_;
  symbolic::Variable y_;
};
}  // namespace
}  // namespace solvers
}  // namespace drake
