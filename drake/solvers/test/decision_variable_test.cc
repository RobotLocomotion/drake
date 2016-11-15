#include "drake/solvers/decision_variable.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
GTEST_TEST(TestDecisionVariable, TestDecisionVariableValue) {
  MathematicalProgram prog;
  auto X1 = prog.AddContinuousVariables(2, 3, std::vector<std::string>(6, "X"));
  EXPECT_EQ(prog.num_vars(), 6);
  auto S1 =
      prog.AddSymmetricContinuousVariables(3, std::vector<std::string>(6, "S"));
  EXPECT_EQ(prog.num_vars(), 12);
  auto x1 = prog.AddContinuousVariables(6, "x");
  EXPECT_EQ(prog.num_vars(), 18);
  std::array<std::string, 6> X_name = {"X", "X", "X", "X", "X", "X"};
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


  EXPECT_TRUE(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(X1), X_expected, 1E-14, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(S1), S_expected, 1E-14, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(x1), x_value, 1E-14, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(X2), X_expected, 1E-14, MatrixCompareType::absolute));

  EXPECT_TRUE(VariableVectorRefContainsColumnVectorsOnly({x1}));
  EXPECT_FALSE(VariableVectorContainsColumnVectorsOnly({X1, S1}));
  for (int i = 0; i < 6; ++i) {
    EXPECT_TRUE(DecisionVariableMatrixCoversIndex(X1, i));
    EXPECT_TRUE(DecisionVariableMatrixCoversIndex(S1, i + 6));
    EXPECT_TRUE(DecisionVariableMatrixCoversIndex(x1, i + 12));
    EXPECT_TRUE(DecisionVariableMatrixCoversIndex(X2, i + 18));
  }

  DecisionVariableMatrix <2, 6> X_assembled;
  X_assembled << X1, X2;
  Eigen::Matrix<double, 2, 6> X_assembled_expected;
  X_assembled_expected << X_expected, X_expected;
  EXPECT_TRUE(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(X_assembled), X_assembled_expected, 1E-10, MatrixCompareType::absolute));
}
}  // namespace solvers
}  // namespace drake
