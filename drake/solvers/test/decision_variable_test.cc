#include "drake/solvers/decision_variable.h"

#include "gtest/gtest.h"
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
  // Constructs a variable size decision variable matrix.
  MatrixXDecisionVariables X(v_immutable.data(), 2, 5);
  for (int j = 0; j < 5; ++j) {
    for (int i = 0; i < 2; ++i) {
      EXPECT_EQ(X(i, j).get().value(), 2 * (j * 2 + i));
    }
  }

  v_mutable[2].set_value(5);
  EXPECT_EQ(v_immutable[2].get().value(), 5);
  EXPECT_EQ(X(0, 1).get().value(), 5);

  // Constructs a fixed size decision variable matrix.
  MatrixDecisionVariables<2, 4> X24(v_immutable.data());

  // Take a column of the matrix, check the value.
  auto X24_col1 = X24.col(1);
  EXPECT_EQ(X24_col1(0).get().value(), 5);

  v_mutable[2].set_value(-1);
  EXPECT_EQ(X24_col1(0).get().value(), -1);
  EXPECT_EQ(X24(0, 1).get().value(), -1);

}
} // namespace solvers
} // namespace drake