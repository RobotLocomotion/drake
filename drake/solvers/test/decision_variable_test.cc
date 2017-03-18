#include "drake/solvers/decision_variable.h"

#include <gtest/gtest.h>

#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace solvers {
namespace test {

using drake::symbolic::test::VarEqual;

GTEST_TEST(TestDecisionVariable, TestVariableListRef) {
  symbolic::Variable x1("x1");
  symbolic::Variable x2("x2");
  symbolic::Variable x3("x3");
  symbolic::Variable x4("x4");

  VectorDecisionVariable<2> x_vec1(x3, x1);
  VectorDecisionVariable<2> x_vec2(x2, x4);
  VariableRefList var_list{x_vec1, x_vec2};

  VectorXDecisionVariable stacked_vars = ConcatenateVariableRefList(var_list);
  EXPECT_EQ(stacked_vars.rows(), 4);
  EXPECT_PRED2(VarEqual, stacked_vars(0), x3);
  EXPECT_PRED2(VarEqual, stacked_vars(1), x1);
  EXPECT_PRED2(VarEqual, stacked_vars(2), x2);
  EXPECT_PRED2(VarEqual, stacked_vars(3), x4);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
