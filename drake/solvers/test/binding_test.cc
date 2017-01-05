#include "drake/solvers/binding.h"

#include "gtest/gtest.h"

namespace drake {
namespace solvers {
namespace test {
GTEST_TEST(TestBinding, constructBinding) {
  symbolic::Variable x1("x1");
  symbolic::Variable x2("x2");
  symbolic::Variable x3("x3");
  auto bb_con = std::make_shared<BoundingBoxConstraint>(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());

  // Checks if the bound variables are stored in the right order.
  Binding binding1(bb_con, {DecisionVariableVector<2>(x3, x1), x2});
  DecisionVariableVector<3> var1_expected(x3, x1, x2);
  for (int i = 0; i < 3; ++i) {
    DRAKE_ASSERT(binding1.variables()(i) == var1_expected(i));
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
