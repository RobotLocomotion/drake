#include "drake/systems/framework/system_constraint.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
  SystemConstraint<double>::CalcCallback calc2 = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    *value =
        Eigen::Vector2d(context.get_continuous_state_vector().CopyToVector());
  };

  Eigen::VectorXd value;

  // Make a (linear) system just to make a valid context.
  LinearSystem<double> system(Eigen::Matrix2d::Identity(),
                              Eigen::Vector2d::Zero(), Eigen::MatrixXd(0, 2),
                              Eigen::MatrixXd(0, 1));
  auto context = system.CreateDefaultContext();

  const double tol = 1e-6;

  // Test equality constraint.
  SystemConstraint<double> equality_constraint(
      calc, 1, SystemConstraintType::kEquality, "equality constraint");
  context->get_mutable_continuous_state_vector().SetAtIndex(1, 5.0);
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_FALSE(equality_constraint.CheckSatisfied(*context, tol));

  context->get_mutable_continuous_state_vector().SetAtIndex(1, 0.0);
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 0.0);
  EXPECT_TRUE(equality_constraint.CheckSatisfied(*context, tol));

  EXPECT_EQ(equality_constraint.size(), 1);
  EXPECT_EQ(equality_constraint.description(), "equality constraint");

  // Test inequality constraint.
  SystemConstraint<double> inequality_constraint(
      calc2, 2, SystemConstraintType::kInequality, "inequality constraint");
  context->get_mutable_continuous_state_vector().SetAtIndex(0, 3.0);
  context->get_mutable_continuous_state_vector().SetAtIndex(1, 5.0);
  inequality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 3.0);
  EXPECT_EQ(value[1], 5.0);
  EXPECT_TRUE(inequality_constraint.CheckSatisfied(*context, tol));

  context->get_mutable_continuous_state_vector().SetAtIndex(1, -0.5);
  inequality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[1], -0.5);
  EXPECT_FALSE(inequality_constraint.CheckSatisfied(*context, tol));

  EXPECT_EQ(inequality_constraint.size(), 2);
  EXPECT_EQ(inequality_constraint.description(), "inequality constraint");
}

}  // namespace
}  // namespace systems
}  // namespace drake
