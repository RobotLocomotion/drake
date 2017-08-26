#include "drake/systems/framework/system_constraint.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

class TestSystem : public LeafSystem<double> {
 public:
  TestSystem() { DeclareContinuousState(2); }

  // Expose some protected methods for testing.
  using LeafSystem<double>::DeclareConstraint;

  void CalcState0Constraint(const Context<double>& context,
                            Eigen::VectorXd* value) const {
    *value = Vector1d(context.get_continuous_state_vector().GetAtIndex(0));
  }
  void CalcStateConstraint(const Context<double>& context,
                           Eigen::VectorXd* value) const {
    *value = context.get_continuous_state_vector().CopyToVector();
  }

 private:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // xdot = -x.
    derivatives->SetFromVector(-dynamic_cast<const BasicVector<double>&>(
                                    context.get_continuous_state_vector())
                                    .get_value());
  }
};

// Tests adding constraints implemented as methods inside the System class.
GTEST_TEST(SystemConstraintTest, ClassMethodTest) {
  TestSystem dut;
  EXPECT_EQ(dut.get_num_constraints(), 0);

  EXPECT_EQ(dut.DeclareConstraint(&TestSystem::CalcState0Constraint,
                                  Vector1d(0.0), Vector1d(0.0), "x0"),
            0);
  EXPECT_EQ(dut.get_num_constraints(), 1);

  EXPECT_EQ(dut.DeclareConstraint(&TestSystem::CalcStateConstraint,
                                  Eigen::Vector2d(0.0, 0.0),
                                  Eigen::Vector2d(8.0, 10.0), "x"),
            1);
  EXPECT_EQ(dut.get_num_constraints(), 2);

  auto context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector()->SetFromVector(
      Eigen::Vector2d(5.0, 7.0));

  EXPECT_EQ(dut.get_constraint(0).size(), 1);
  EXPECT_EQ(dut.get_constraint(1).size(), 2);

  Eigen::VectorXd value;
  dut.get_constraint(0).Calc(*context, &value);
  EXPECT_EQ(value.rows(), 1);
  EXPECT_EQ(value[0], 5.0);

  dut.get_constraint(1).Calc(*context, &value);
  EXPECT_EQ(value.rows(), 2);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_EQ(value[1], 7.0);

  EXPECT_EQ(dut.get_constraint(0).lower_bound()[0], 0.0);
  EXPECT_EQ(dut.get_constraint(0).upper_bound()[0], 0.0);
  EXPECT_EQ(dut.get_constraint(0).description(), "x0");

  EXPECT_EQ(dut.get_constraint(1).lower_bound()[1], 0.0);
  EXPECT_EQ(dut.get_constraint(1).upper_bound()[1], 10.0);
  EXPECT_EQ(dut.get_constraint(1).description(), "x");
}

// Tests adding constraints implemented as function handles (lambda functions).
GTEST_TEST(SystemConstraintTest, FunctionHandleTest) {
  TestSystem dut;
  EXPECT_EQ(dut.get_num_constraints(), 0);

  SystemConstraint<double>::CalcCallback calc = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    *value = Vector1d(context.get_continuous_state_vector().GetAtIndex(1));
  };
  EXPECT_EQ(dut.DeclareConstraint(calc, Vector1d(0.0), Vector1d(2.0), "x1"), 0);
  EXPECT_EQ(dut.get_num_constraints(), 1);

  auto context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector()->SetFromVector(
      Eigen::Vector2d(5.0, 7.0));

  Eigen::VectorXd value;
  dut.get_constraint(0).Calc(*context, &value);
  EXPECT_EQ(value.rows(), 1);
  EXPECT_EQ(value[0], 7.0);

  EXPECT_EQ(dut.get_constraint(0).description(), "x1");
}

GTEST_TEST(SystemConstraintTest, DiagramTest) {
  systems::DiagramBuilder<double> builder;
  auto sys1 = builder.AddSystem<TestSystem>();

  auto sys2 = builder.AddSystem<TestSystem>();
  sys2->DeclareConstraint(&TestSystem::CalcState0Constraint, Vector1d(0.0),
                          Vector1d(0.0), "sys2x0");
  sys2->DeclareConstraint(&TestSystem::CalcStateConstraint,
                          Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(8.0, 10.0),
                          "x");

  auto diagram = builder.Build();
  EXPECT_EQ(diagram->get_num_constraints(), 2);

  auto context = diagram->CreateDefaultContext();

  // Set sys1 context.
  diagram->GetMutableSubsystemContext(*sys1, context.get())
      .get_mutable_continuous_state_vector()
      ->SetFromVector(Eigen::Vector2d(5.0, 7.0));

  // Set sys2 context.
  diagram->GetMutableSubsystemContext(*sys2, context.get())
      .get_mutable_continuous_state_vector()
      ->SetFromVector(Eigen::Vector2d(11.0, 12.0));

  Eigen::VectorXd value;
  diagram->get_constraint(0).Calc(*context, &value);
  EXPECT_EQ(value.size(), 1);
  EXPECT_EQ(value[0], 11.0);
  EXPECT_EQ(diagram->get_constraint(0).lower_bound()[0], 0.0);
  EXPECT_EQ(diagram->get_constraint(0).upper_bound()[0], 0.0);

  diagram->get_constraint(1).Calc(*context, &value);
  EXPECT_EQ(value.size(), 2);
  EXPECT_EQ(value[0], 11.0);
  EXPECT_EQ(value[1], 12.0);
  EXPECT_EQ(diagram->get_constraint(1).lower_bound()[1], 0.0);
  EXPECT_EQ(diagram->get_constraint(1).upper_bound()[1], 10.0);
}

}  // namespace
}  // namespace systems
}  // namespace drake
