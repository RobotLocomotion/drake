#include "drake/solvers/decision_variable.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
GTEST_TEST(TestDecisionVariable, TestConstructor) {
  DecisionVariableScalar x(DecisionVariableScalar::VarType::CONTINUOUS, "x", 0);
  EXPECT_EQ(x.type(), DecisionVariableScalar::VarType::CONTINUOUS);
  EXPECT_EQ(static_cast<int>(x.index()), 0);

  // Now create a vector of decision variables.
  std::vector<DecisionVariableScalar> v_mutable;
  std::vector<std::reference_wrapper<const DecisionVariableScalar>> v_immutable;
  v_mutable.reserve(10);
  v_immutable.reserve(10);
  for (int i = 0; i < 10; ++i) {
    DecisionVariableScalar vi(DecisionVariableScalar::VarType::CONTINUOUS, "x", i);
    v_mutable.push_back(DecisionVariableScalar(DecisionVariableScalar::VarType::CONTINUOUS, "x", i));
    v_immutable.push_back(std::cref(v_mutable[i]));
    v_mutable[i].set_value(2*i);
    EXPECT_EQ(v_immutable[i].get().value(), 2 * i);
  }

  // Constructs a MatrixDecisionVariable from v_immutable.
  MatrixDecisionVariable X(2, 5, v_immutable);
  // Check value(i, j) function, and overloaded operator (i, j).
  for (int j = 0; j < 5; ++j) {
    for (int i = 0; i < 2; ++i) {
      EXPECT_EQ(X.value(i, j), 2 * (j * 2 + i));
      EXPECT_EQ(X(i, j).value(0, 0), 2 * (j * 2 + i));
      EXPECT_EQ(X.index(i, j), j * 2 + i);
    }
  }
  // Check value() function.
  Eigen::MatrixXd mat_value = X.value();
  Eigen::Matrix<double, 2, 5> mat_value_expected;
  mat_value_expected << 0, 4, 8, 12, 16,
                        2, 6, 10, 14, 18;
  EXPECT_TRUE(CompareMatrices(mat_value, mat_value_expected, 1E-10, MatrixCompareType::absolute));

  // Check block() function
  const MatrixDecisionVariable& X_block = X.block(0, 1, 2, 2);
  Eigen::MatrixXd mat_block_value = X_block.value();
  EXPECT_TRUE(CompareMatrices(mat_block_value, mat_value_expected.block(0, 1, 2, 2), 1E-10, MatrixCompareType::absolute));

  v_mutable[2].set_value(5);
  EXPECT_EQ(v_immutable[2].get().value(), 5);
  EXPECT_EQ(X(0, 1).value(0, 0), 5);
  EXPECT_EQ(X.value(0, 1), 5);
  mat_block_value = X_block.value();
  mat_value_expected(0, 1) = 5;
  EXPECT_TRUE(CompareMatrices(mat_block_value, mat_value_expected.block(0, 1, 2, 2), 1E-10, MatrixCompareType::absolute));
}
} // namespace solvers
} // namespace drake