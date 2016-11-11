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
  Eigen::Matrix<double, 6, 1> x_value;
  x_value << 0, 2, 4, 6, 8, 10;
  Eigen::Matrix<double, 6, 1> s_value;
  s_value << 0, -2, -4, -6, -8, -10;
  Eigen::Matrix<double, 3, 3> S_expected;
  S_expected << 0, -2, -4, -2, -6, -8, -4, -8, -10;
  Eigen::Matrix<double, 18, 1> var_values;
  var_values << x_value, s_value, x_value;
  prog.SetDecisionVariableValues(var_values);
  Eigen::MatrixXd X_expected = x_value;
  X_expected.resize(2, 3);

  DRAKE_ASSERT(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(X1), X_expected, 1E-14, MatrixCompareType::absolute));
  DRAKE_ASSERT(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(S1), S_expected, 1E-14, MatrixCompareType::absolute));
  DRAKE_ASSERT(CompareMatrices(DecisionVariableMatrixToDoubleMatrix(x1), x_value, 1E-14, MatrixCompareType::absolute));
}
}  // namespace solvers
}  // namespace drake
