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
  v_mutable.reserve(6);
  v_immutable.reserve(6);
  for (int i = 0; i < 6; ++i) {
    DecisionVariableScalar vi(DecisionVariableScalar::VarType::CONTINUOUS, "x",
                              i);
    v_mutable.push_back(DecisionVariableScalar(
        DecisionVariableScalar::VarType::CONTINUOUS, "x", i));
    v_immutable.push_back(std::cref(v_mutable[i]));
    v_mutable[i].set_value(2 * i);
    EXPECT_EQ(v_immutable[i].get().value(), 2 * i);
  }

  // Constructs a non-symmetric MatrixDecisionVariable from v_immutable.
  MatrixDecisionVariable X(2, 3, v_immutable, false);
  EXPECT_FALSE(X.is_symmetric());
  EXPECT_EQ(X.NumberOfVariables(), 6);
  EXPECT_EQ(X.rows(), 2);
  EXPECT_EQ(X.cols(), 3);
  // Constructs a symmetric 3 x 3 MatrixDecisionVariable from v_immutable.
  MatrixDecisionVariable S(3, 3, v_immutable, true);
  EXPECT_TRUE(S.is_symmetric());
  EXPECT_EQ(S.NumberOfVariables(), 6);
  EXPECT_EQ(S.rows(), 3);
  EXPECT_EQ(S.cols(), 3);
  // Check value(i, j) function, and overloaded operator (i, j).
  Eigen::Matrix<double, 2, 3> mat_value_expected;
  mat_value_expected << 0, 4, 8, 2, 6, 10;
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 2; ++i) {
      EXPECT_EQ(X.value(i, j), mat_value_expected(i, j));
      EXPECT_EQ(X(i, j).value(0, 0), mat_value_expected(i, j));
      EXPECT_EQ(X.index(i, j), j * 2 + i);
    }
  }
  // Check value(i, j) and overloaded operator (i, j) for a symmetric matrix.
  Eigen::Matrix3d symmetric_matrix_expected;
  symmetric_matrix_expected << 0, 2, 4, 2, 6, 8, 4, 8, 10;
  Eigen::Matrix3i symmetric_matrix_index;
  symmetric_matrix_index << 0, 1, 2, 1, 3, 4, 2, 4, 5;
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(S.value(i, j), symmetric_matrix_expected(i, j));
      EXPECT_EQ(S(i, j).value(0, 0), symmetric_matrix_expected(i, j));
      EXPECT_EQ(S.index(i, j), symmetric_matrix_index(i, j));
    }
  }
  // Check value() function.
  Eigen::MatrixXd mat_value = X.value();
  EXPECT_TRUE(CompareMatrices(mat_value, mat_value_expected, 1E-10,
                              MatrixCompareType::absolute));
  Eigen::MatrixXd symmetric_mat_value = S.value();
  EXPECT_TRUE(CompareMatrices(symmetric_mat_value, symmetric_matrix_expected,
                              1E-10, MatrixCompareType::absolute));

  // Check block() function
  const MatrixDecisionVariable& X_block = X.block(0, 1, 2, 2);
  Eigen::MatrixXd mat_block_value = X_block.value();
  EXPECT_TRUE(CompareMatrices(mat_block_value,
                              mat_value_expected.block(0, 1, 2, 2), 1E-10,
                              MatrixCompareType::absolute));

  // Change value of the referenced decision variable.
  double new_value = 5;
  v_mutable[2].set_value(new_value);
  EXPECT_EQ(v_immutable[2].get().value(), new_value);
  EXPECT_EQ(X(0, 1).value(0, 0), new_value);
  EXPECT_EQ(X.value(0, 1), new_value);
  EXPECT_EQ(S(2, 0).value(0, 0), new_value);
  EXPECT_EQ(S(0, 2).value(0, 0), new_value);
  EXPECT_EQ(S.value(0, 2), new_value);
  EXPECT_EQ(S.value(2, 0), new_value);
  mat_block_value = X_block.value();
  mat_value_expected(0, 1) = new_value;
  EXPECT_TRUE(CompareMatrices(mat_block_value,
                              mat_value_expected.block(0, 1, 2, 2), 1E-10,
                              MatrixCompareType::absolute));
  symmetric_matrix_expected(2, 0) = new_value;
  symmetric_matrix_expected(0, 2) = new_value;
  EXPECT_TRUE(CompareMatrices(S.value(), symmetric_matrix_expected, 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(S.row(2).value(),
                              symmetric_matrix_expected.row(2), 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(S.col(0).value(),
                              symmetric_matrix_expected.col(0), 1E-10,
                              MatrixCompareType::absolute));
}
}  // namespace solvers
}  // namespace drake
