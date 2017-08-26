#include "drake/systems/framework/system_constraint.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace {

// Just a simple test to call each of the public methods (all non-trivial
// argument checks are enforced with DRAKE_DEMAND, so are not tested here).
GTEST_TEST(SystemConstraintTest, BasicTest) {
  SystemConstraint<double>::CalcCallback calc = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    *value = Vector1d(context.get_continuous_state_vector().GetAtIndex(1));
  };
  SystemConstraint<double> constraint(calc, Vector1d(-1.0), Vector1d(1.0),
                                      "test constraint");
  Eigen::VectorXd value;

  // Make a (linear) system just to make a valid context.
  LinearSystem<double> system(Eigen::Matrix2d::Identity(),
                              Eigen::Vector2d::Zero(), Eigen::MatrixXd(0, 2),
                              Eigen::MatrixXd(0, 1));
  auto context = system.CreateDefaultContext();

  context->get_mutable_continuous_state_vector()->SetAtIndex(1, 5.0);
  constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_FALSE(constraint.CheckSatisfied(*context));

  context->get_mutable_continuous_state_vector()->SetAtIndex(1, 0.5);
  constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 0.5);
  EXPECT_TRUE(constraint.CheckSatisfied(*context));

  EXPECT_EQ(constraint.size(), 1);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), Vector1d(-1.0)));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(1.0)));
  EXPECT_EQ(constraint.description(), "test constraint");
}

}  // namespace
}  // namespace systems
}  // namespace drake
