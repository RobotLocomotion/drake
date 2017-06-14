#include "drake/solvers/mixed_integer_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(TestMixedIntegerUtil, TestCeilLog2) {
  const int kMaxExponent = 15;
  std::array<int, kMaxExponent + 1> two_to_exponent;
  two_to_exponent[0] = 1;
  for (int i = 1; i <= kMaxExponent; ++i) {
    two_to_exponent[i] = 2 * two_to_exponent[i - 1];
  }
  EXPECT_EQ(0, CeilLog2(1));
  for (int i = 0; i < kMaxExponent; ++i) {
    for (int j = two_to_exponent[i] + 1; j <= two_to_exponent[i]; ++j) {
      EXPECT_EQ(i + 1, CeilLog2(j));
    }
  }
}

GTEST_TEST(TestGrayCode, TestCalculateGrayCodes) {
  for (int i = 0; i < 4; i++) {
    auto test_code = drake::solvers::internal::CalculateReflectedGrayCodes(i);
    // Asking for codes for 0 bits should generate 0 for 0 bits.
    // Asking for codes for i bits should generate 2^(i) codes for i bits.
    EXPECT_EQ(test_code.cols(), i);
    EXPECT_EQ(test_code.rows(), i == 0 ? 0 : 1 << i);
    // Each code should differ by only one bit from the previous code.
    for (int j = 1; j < test_code.rows(); j ++) {
      EXPECT_EQ((test_code.row(j) - test_code.row(j - 1)).cwiseAbs().sum(), 1);
    }
  }
}

GTEST_TEST(TestGrayCode, TestGrayCodeToInteger) {
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(0, 0)), 0);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(0, 1)), 1);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(1, 1)), 2);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector2i(1, 0)), 3);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 0, 0)), 0);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 0, 1)), 1);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 1, 1)), 2);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(0, 1, 0)), 3);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 1, 0)), 4);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 1, 1)), 5);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 0, 1)), 6);
  EXPECT_EQ(drake::solvers::internal::GrayCodeToInteger(Eigen::Vector3i(1, 0, 0)), 7);
}

GTEST_TEST(TestMixedIntegerUtil, TestLogarithmicSOS2) {
  // Solve the program
  // min λᵀ * λ
  // s.t sum λ = 1
  //     λ >= 0
  //     λ in sos2
  // We loop over i such that only λ(i) and λ(i+1) can be non-zero.
  // The optimal cost is λ(i) = λ(i + 1) = 0.5.
  MathematicalProgram prog;
  auto lambda = prog.NewContinuousVariables<5>();
  prog.AddBoundingBoxConstraint(0, 1, lambda);
  prog.AddLinearConstraint(lambda.cast<symbolic::Expression>().sum() == 1);

  prog.AddCost(lambda.cast<symbolic::Expression>().dot(lambda.cast<symbolic::Expression>()));

  auto y = AddLogarithmicSOS2Constraint(&prog, lambda.cast<symbolic::Expression>());
  auto y_assignment0 = prog.AddLinearConstraint(y(0) == 0);
  auto y_assignment1 = prog.AddLinearConstraint(y(1) == 0);
  const auto gray_codes = drake::solvers::internal::CalculateReflectedGrayCodes(2);
  for (int i = 0; i < lambda.rows() - 1; ++i) {
    y_assignment0.constraint()->UpdateLowerBound(Vector1d(gray_codes(i, 0)));
    y_assignment0.constraint()->UpdateUpperBound(Vector1d(gray_codes(i, 0)));
    y_assignment1.constraint()->UpdateLowerBound(Vector1d(gray_codes(i, 1)));
    y_assignment1.constraint()->UpdateUpperBound(Vector1d(gray_codes(i, 1)));

    GurobiSolver gurobi_solver;
    auto result = gurobi_solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    const auto lambda_val = prog.GetSolution(lambda);
    Eigen::Matrix<double, 5, 1> lambda_val_expected = Eigen::Matrix<double, 5, 1>::Zero();
    lambda_val_expected(i) = 0.5;
    lambda_val_expected(i + 1) = 0.5;
    EXPECT_TRUE(CompareMatrices(lambda_val, lambda_val_expected, 1E-5, MatrixCompareType::absolute));
  }
}
}  // namespace
}  // namespace solvers
}  // namespace drake