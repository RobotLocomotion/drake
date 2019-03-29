#include "drake/systems/primitives/symbolic_vector_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;
using Eigen::Vector4d;
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
      {}, Vector0<Variable>{}, Vector1<Variable>{u}, Vector0<Expression>{},
      Vector1<Expression>{u});

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
  SymbolicVectorSystem<double> system(
      {}, Vector0<Variable>{}, u, Vector0<Expression>{}, u.cast<Expression>());

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
      t, Vector0<Variable>{}, Vector0<Variable>{}, Vector0<Expression>{},
      Vector1<Expression>{2. * t});

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->is_stateless());
  EXPECT_EQ(system.get_num_input_ports(), 0);
  EXPECT_EQ(system.get_output_port().size(), 1);

  context->SetTime(2.0);
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

GTEST_TEST(SymbolicVectorSystemTest, DiscreteStateOnly) {
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
  EXPECT_TRUE(CompareMatrices(discrete_variables->get_vector().get_value(),
                              Vector1d{-xval + xval * xval * xval}));
}

GTEST_TEST(SymbolicVectorSystemTest, ContinuousTimeSymbolic) {
  Variable t("t");
  Vector2<Variable> x{Variable{"x0"}, Variable{"x1"}};
  Vector2<Variable> u{Variable{"u0"}, Variable{"u1"}};
  auto system = SymbolicVectorSystemBuilder()
                    .time(t)
                    .state(x)
                    .input(u)
                    .dynamics(Vector2<Expression>{t, x[1] + u[1]})
                    .output(Vector2<Expression>{x[0] + u[0], t})
                    .Build<Expression>();

  auto context = system->CreateDefaultContext();

  Variable tc("tc");
  Vector2<Expression> xc{Variable{"xc0"}, Variable{"xc1"}};
  Vector2<Expression> uc{Variable{"uc0"}, Variable{"uc1"}};

  context->SetTime(tc);
  context->SetContinuousState(xc);
  context->FixInputPort(0, uc);

  const auto& xdot = system->EvalTimeDerivatives(*context).get_vector();
  EXPECT_TRUE(xdot.GetAtIndex(0).EqualTo(tc));
  EXPECT_TRUE(xdot.GetAtIndex(1).EqualTo(xc[1] + uc[1]));

  const auto& y = system->get_output_port()
                      .template Eval<BasicVector<Expression>>(*context)
                      .get_value();
  EXPECT_TRUE(y[0].EqualTo(xc[0] + uc[0]));
  EXPECT_TRUE(y[1].EqualTo(tc));
}

GTEST_TEST(SymbolicVectorSystemTest, DiscreteTimeSymbolic) {
  Variable t("t");
  Vector2<Variable> x{Variable{"x0"}, Variable{"x1"}};
  Vector2<Variable> u{Variable{"u0"}, Variable{"u1"}};
  auto system = SymbolicVectorSystemBuilder()
                    .time(t)
                    .state(x)
                    .input(u)
                    .dynamics(Vector2<Expression>{t, x[1] + u[1]})
                    .output(Vector2<Expression>{x[0] + u[0], t})
                    .time_period(1.0)
                    .Build<Expression>();

  auto context = system->CreateDefaultContext();

  Variable tc("tc");
  Vector2<Expression> xc{Variable{"xc0"}, Variable{"xc1"}};
  Vector2<Expression> uc{Variable{"uc0"}, Variable{"uc1"}};

  context->SetTime(tc);
  context->get_mutable_discrete_state_vector().SetFromVector(xc);
  context->FixInputPort(0, uc);

  auto discrete_variables = system->AllocateDiscreteVariables();
  system->CalcDiscreteVariableUpdates(*context, discrete_variables.get());
  const auto& xnext = discrete_variables->get_vector().get_value();
  EXPECT_TRUE(xnext[0].EqualTo(tc));
  EXPECT_TRUE(xnext[1].EqualTo(xc[1] + uc[1]));

  const auto& y = system->get_output_port()
                      .template Eval<BasicVector<Expression>>(*context)
                      .get_value();
  EXPECT_TRUE(y[0].EqualTo(xc[0] + uc[0]));
  EXPECT_TRUE(y[1].EqualTo(tc));
}

GTEST_TEST(SymbolicVectorSystemTest, TestScalarConversion) {
  Variable x{"x"};
  auto system = SymbolicVectorSystemBuilder().state(x).dynamics(-x).Build();

  EXPECT_TRUE(is_autodiffxd_convertible(*system));
  EXPECT_TRUE(is_symbolic_convertible(*system));
}

GTEST_TEST(SymbolicVectorSystemTest, TestAutodiffXd) {
  Variable t{"t"};
  Variable x{"x"};
  Vector2<Variable> u{Variable{"u0"}, Variable{"u1"}};
  auto system = SymbolicVectorSystemBuilder()
                    .time(t)
                    .state(x)
                    .input(u)
                    .dynamics(-x + pow(x, 3))
                    .output(t + 2. * u[0] + 3. * u[1])
                    .Build();

  auto autodiff_system = system->ToAutoDiffXd();
  auto context = autodiff_system->CreateDefaultContext();

  const double tval = 1.23;
  const double xval = 0.45;
  const Vector2d uval{5.6, 7.8};
  Vector4d txuval;
  txuval << tval, xval, uval;

  const VectorX<AutoDiffXd> txu = math::initializeAutoDiff(txuval);

  context->SetTime(txu[0]);
  context->SetContinuousState(txu.segment<1>(1));
  context->FixInputPort(0, txu.tail<2>());
  const auto xdotval =
      autodiff_system->EvalTimeDerivatives(*context).CopyToVector();

  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(xdotval),
                              Vector1d{-xval + xval * xval * xval}));
  EXPECT_TRUE(CompareMatrices(xdotval[0].derivatives(),
                              Vector4d{0, -1. + 3. * xval * xval, 0, 0}));

  const auto& yval = autodiff_system->get_output_port(0)
                         .template Eval<BasicVector<AutoDiffXd>>(*context)
                         .get_value();
  EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(yval),
                              Vector1d{tval + 2 * uval[0] + 3 * uval[1]}));
  EXPECT_TRUE(CompareMatrices(yval[0].derivatives(), Vector4d{1., 0, 2., 3.}));
}

}  // namespace
}  // namespace systems
}  // namespace drake
