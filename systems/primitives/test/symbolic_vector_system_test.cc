#include "drake/systems/primitives/symbolic_vector_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;
using Eigen::VectorXd;
using symbolic::Expression;
using symbolic::Variable;

GTEST_TEST(SymbolicVectorSystemBuilderTest, BasicPattern) {
  Variable t("t");
  Vector2<Variable> x{Variable{"x0"}, Variable{"x1"}};
  Vector2<Variable> u{Variable{"u0"}, Variable{"u1"}};

  EXPECT_TRUE(SymbolicVectorSystemBuilder().time(t).time()->equal_to(t));

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().state(x[1]).state()[0].equal_to(x[1]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder().state(x).state()[1].equal_to(x[1]));

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().input(u[1]).input()[0].equal_to(u[1]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder().input(u).input()[1].equal_to(u[1]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .dynamics(x[0] + u[0])
                  .dynamics()[0]
                  .EqualTo(x[0] + u[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .dynamics(Vector1<Expression>{x[0] + u[0]})
                  .dynamics()[0]
                  .EqualTo(x[0] + u[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .output(x[0] + u[0])
                  .output()[0]
                  .EqualTo(x[0] + u[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .output(Vector1<Expression>{x[0] + u[0]})
                  .output()[0]
                  .EqualTo(x[0] + u[0]));

  EXPECT_EQ(SymbolicVectorSystemBuilder().time_period(3.2).time_period(), 3.2);

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .state(x[0])
                  .dynamics(-x[0] + pow(x[0], 3))
                  .state()[0]
                  .equal_to(x[0]));
}

// This test will fail on valgrind if we have dangling references.
GTEST_TEST(SymbolicVectorSystemTest, DanglingReferenceTest) {
  Vector2<Variable> x{Variable{"x0"}, Variable{"x1"}};

  SymbolicVectorSystemBuilder builder1;
  builder1.state(x);
  EXPECT_TRUE(builder1.state()[0].equal_to(x[0]));

  const auto& builder2 = SymbolicVectorSystemBuilder().state(x);
  EXPECT_TRUE(builder2.state()[0].equal_to(x[0]));
}

GTEST_TEST(SymbolicVectorSystemTest, CubicPolyViaBuilder) {
  Variable x{"x"};
  auto system = SymbolicVectorSystemBuilder()
                    .state(x)
                    .dynamics(-x + pow(x, 3))
                    .output(x)
                    .Build();

  auto context = system->CreateDefaultContext();
  EXPECT_TRUE(context->has_only_continuous_state());
  EXPECT_EQ(context->get_num_total_states(), 1);
  EXPECT_EQ(system->get_num_input_ports(), 0);
  EXPECT_EQ(system->get_num_output_ports(), 1);

  double xval = 0.45;
  context->SetContinuousState(Vector1d{xval});
  const auto& xdotval = system->EvalTimeDerivatives(*context);
  EXPECT_TRUE(CompareMatrices(xdotval.CopyToVector(),
                              Vector1d{-xval + xval * xval * xval}));
  EXPECT_TRUE(CompareMatrices(system->get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector1d{xval}));
}

GTEST_TEST(SymbolicVectorSystemTest, ScalarPassThrough) {
  // y = u.
  Variable u("u");
  SymbolicVectorSystem<double> system(
      {}, Vector0<Variable>{}, Vector1<Variable>{u},
      Vector0<Expression>{}, Vector1<Expression>{u});

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->is_stateless());
  EXPECT_EQ(system.get_input_port().size(), 1);
  EXPECT_EQ(system.get_output_port().size(), 1);

  context->FixInputPort(0, Vector1d{0.12});
  EXPECT_TRUE(CompareMatrices(system.get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector1d{0.12}));
}

GTEST_TEST(SymbolicVectorSystemTest, VectorPassThrough) {
  // y = u.
  Vector2<Variable> u{Variable{"u0"}, Variable{"u1"}};
  SymbolicVectorSystem<double> system({}, Vector0<Variable>{}, u,
                                      Vector0<Expression>{},
                                      u.cast<Expression>());

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->is_stateless());
  EXPECT_EQ(system.get_input_port().size(), 2);
  EXPECT_EQ(system.get_output_port().size(), 2);

  context->FixInputPort(0, Vector2d{0.12, 0.34});
  EXPECT_TRUE(CompareMatrices(system.get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector2d{0.12, 0.34}));
}

GTEST_TEST(SymbolicVectorSystemTest, OutputScaledTime) {
  Variable t("t");
  SymbolicVectorSystem<double> system(
      t, Vector0<Variable>{}, Vector0<Variable>{},
      Vector0<Expression>{}, Vector1<Expression>{2. * t});

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->is_stateless());
  EXPECT_EQ(system.get_num_input_ports(), 0);
  EXPECT_EQ(system.get_output_port().size(), 1);

  context->set_time(2.0);
  EXPECT_TRUE(CompareMatrices(system.get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector1d{4.0}));
}

GTEST_TEST(SymbolicVectorSystemTest, ContinuousStateOnly) {
  // xdot = -x + x^3
  Variable x{"x"};
  SymbolicVectorSystem<double> system(
      {}, Vector1<Variable>{x}, Vector0<Variable>{},
      Vector1<Expression>{-x + pow(x, 3)}, Vector0<Expression>{});

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->has_only_continuous_state());
  EXPECT_EQ(context->get_num_total_states(), 1);
  EXPECT_EQ(system.get_num_input_ports(), 0);
  EXPECT_EQ(system.get_num_output_ports(), 0);

  double xval = 0.45;
  context->SetContinuousState(Vector1d{xval});
  const auto& xdotval = system.EvalTimeDerivatives(*context);
  EXPECT_TRUE(CompareMatrices(xdotval.CopyToVector(),
                              Vector1d{-xval + xval * xval * xval}));
}

GTEST_TEST(SymbolicVectorSystemTest, DiscreteStateOnlyTest) {
  // xnext = -x + x^3
  Variable x{"x"};
  SymbolicVectorSystem<double> system(
      {}, Vector1<Variable>{x}, Vector0<Variable>{},
      Vector1<Expression>{-x + pow(x, 3)}, Vector0<Expression>{}, 0.1);

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->has_only_discrete_state());
  EXPECT_EQ(context->get_num_total_states(), 1);
  EXPECT_EQ(system.get_num_input_ports(), 0);
  EXPECT_EQ(system.get_num_output_ports(), 0);

  double xval = 0.45;
  context->get_mutable_discrete_state_vector()[0] = xval;
  auto discrete_variables = system.AllocateDiscreteVariables();
  system.CalcDiscreteVariableUpdates(*context, discrete_variables.get());
  EXPECT_TRUE(CompareMatrices(discrete_variables->get_vector().CopyToVector(),
                              Vector1d{-xval + xval * xval * xval}));
}

}  // namespace
}  // namespace systems
}  // namespace drake
