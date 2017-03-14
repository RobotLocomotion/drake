#include "drake/solvers/binding.h"

#include "gtest/gtest.h"

#include "drake/common/test/symbolic_test_util.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {
namespace test {

using drake::symbolic::test::VarEqual;

GTEST_TEST(TestBinding, constructBinding) {
  symbolic::Variable x1("x1");
  symbolic::Variable x2("x2");
  symbolic::Variable x3("x3");
  auto bb_con = std::make_shared<BoundingBoxConstraint>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());

  // Checks if the bound variables are stored in the right order.
  Binding<BoundingBoxConstraint> binding1(
      bb_con,
      {VectorDecisionVariable<2>(x3, x1), VectorDecisionVariable<1>(x2)});
  EXPECT_EQ(binding1.GetNumElements(), 3u);
  VectorDecisionVariable<3> var1_expected(x3, x1, x2);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding1.variables()(i), var1_expected(i));
  }

  // Creates a binding with a single VectorDecisionVariable.
  Binding<BoundingBoxConstraint> binding2(
      bb_con, VectorDecisionVariable<3>(x3, x1, x2));
  EXPECT_EQ(binding2.GetNumElements(), 3u);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(binding2.variables()(i), var1_expected(i));
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
