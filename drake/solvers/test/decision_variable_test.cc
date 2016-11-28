#include "drake/solvers/decision_variable.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/*
 * Test adding decision variables, constructing VariableList, together with
 * functions in DecisionVariableScalar and VariableList.
 */
GTEST_TEST(TestDecisionVariable, TestDecisionVariableValue) {
  MathematicalProgram prog;
  auto X1 = prog.AddContinuousVariables(2, 3, std::vector<std::string>(6, "X"));
  EXPECT_EQ(prog.num_vars(), 6);
  auto S1 =
      prog.AddSymmetricContinuousVariables(3, std::vector<std::string>(6, "S"));
  EXPECT_EQ(prog.num_vars(), 12);
  auto x1 = prog.AddContinuousVariables(6, "x");
  EXPECT_EQ(prog.num_vars(), 18);
  std::array<std::string, 6> X_name = {{"X", "X", "X", "X", "X", "X"}};
  auto X2 = prog.AddContinuousVariables<2, 3>(X_name);
  EXPECT_EQ(prog.num_vars(), 24);
  Eigen::Matrix<double, 6, 1> x_value;
  x_value << 0, 2, 4, 6, 8, 10;
  Eigen::Matrix<double, 6, 1> s_value;
  s_value << 0, -2, -4, -6, -8, -10;
  Eigen::Matrix<double, 3, 3> S_expected;
  S_expected << 0, -2, -4, -2, -6, -8, -4, -8, -10;
  Eigen::Matrix<double, 24, 1> var_values;
  var_values << x_value, s_value, x_value, x_value;
  prog.SetDecisionVariableValues(var_values);
  Eigen::MatrixXd X_expected = x_value;
  X_expected.resize(2, 3);

  // Test if the values in the decision variables are correct.
  EXPECT_TRUE(CompareMatrices(GetSolution(X1),
                              X_expected, 1E-14, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(GetSolution(S1),
                              S_expected, 1E-14, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(GetSolution(x1), x_value,
                              1E-14, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(GetSolution(X2),
                              X_expected, 1E-14, MatrixCompareType::absolute));

  // Test constructing VariableList.
  VariableList var_list1({X1, S1});
  EXPECT_FALSE(var_list1.column_vectors_only());
  VariableList var_list2({x1});
  EXPECT_TRUE(var_list2.column_vectors_only());
  for (int i = 0; i < 6; ++i) {
    EXPECT_TRUE(DecisionVariableMatrixContainsIndex(X1, i));
    EXPECT_TRUE(DecisionVariableMatrixContainsIndex(S1, i + 6));
    EXPECT_TRUE(DecisionVariableMatrixContainsIndex(x1, i + 12));
    EXPECT_TRUE(DecisionVariableMatrixContainsIndex(X2, i + 18));
  }

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_EQ(x1(i) == x1(j), i == j);
    }
  }
  DecisionVariableMatrix<2, 6> X_assembled;
  X_assembled << X1, X2;
  Eigen::Matrix<double, 2, 6> X_assembled_expected;
  X_assembled_expected << X_expected, X_expected;
  EXPECT_TRUE(CompareMatrices(GetSolution(X_assembled),
                              X_assembled_expected, 1E-10,
                              MatrixCompareType::absolute));

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_TRUE(X_assembled(i, j) == X1(i, j));
      EXPECT_TRUE(X_assembled(i, j + 3) == X2(i, j));
    }
  }

  // Test size() and num_unique_variables() functions of VariableList.
  EXPECT_EQ(VariableList({X1}).num_unique_variables(), 6);
  EXPECT_EQ(VariableList({X1}).size(), 6);
  EXPECT_EQ(VariableList({X1, X1}).num_unique_variables(), 6);
  EXPECT_EQ(VariableList({X1, X1}).size(), 12);
  EXPECT_EQ(VariableList({X1, X1.row(1)}).num_unique_variables(), 6);
  EXPECT_EQ(VariableList({X1, X1.row(1)}).size(), 9);

  std::unordered_set<DecisionVariableScalar, DecisionVariableScalarHash>
      X1_unique_variables_expected;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 3; ++j) {
      X1_unique_variables_expected.insert(X1(i, j));
    }
  }
  EXPECT_EQ(VariableList({X1}).unique_variables(),
            X1_unique_variables_expected);
  EXPECT_EQ(VariableList({X1, X1.row(1)}).unique_variables(),
            X1_unique_variables_expected);
}
}  // namespace solvers
}  // namespace drake
