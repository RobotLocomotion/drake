#include "drake/solvers/decision_variable.h"

#include "gtest/gtest.h"
namespace drake {
namespace solvers {
GTEST_TEST(TestDecisionVariable, TestConstructor) {
  DecisionVariableScalar x(DecisionVariableScalar::VarType::CONTINUOUS, "x", 0);
  EXPECT_EQ(x.type(), DecisionVariableScalar::VarType::CONTINUOUS);
  EXPECT_EQ(static_cast<int>(x.index()), 0);

  // Now create a vector of decision variables.
  std::vector<DecisionVariableScalar> v;
  v.reserve(10);
  for (int i = 0; i < 10; ++i) {
    v.push_back(DecisionVariableScalar(DecisionVariableScalar::VarType::CONTINUOUS, "x", i));
    v[i].set_value(2*i);
    EXPECT_EQ(v[i].value(), 2 * i);
  }

  MatrixDecisionVariables X(2, 5, v);
  for (int j = 0; j < 5; ++j) {
    for (int i = 0; i < 2; ++i) {
      EXPECT_EQ(X.value(i, j), 2 * (j * 2 + i));
      EXPECT_EQ(X(i,j).value(0, 0), 2 * (j * 2 + i));
    }
  }

  /*auto X_row1 = X.row(1);
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(X_row1.value(0, j), 2 * (j * 2 + 1));
  }*/
  // Now modify v
  v[3].set_value(2);
  EXPECT_EQ(v[3].value(), 2);
  EXPECT_EQ(X.value(1, 1), 2);
  EXPECT_EQ(X(1, 1).value(0, 0), 2);
  //EXPECT_EQ(X_row1.value(0, 1), 2);
}
} // namespace solvers
} // namespace drake