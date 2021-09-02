#include "drake/systems/primitives/symbolic_vector_system.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using symbolic::Environment;
using symbolic::Expression;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;
using symbolic::test::ExprEqual;
using symbolic::test::PolyEqual;

class SymbolicVectorSystemTest : public ::testing::Test {
 protected:
  const Variable t_{"t"};
  const Vector2<Variable> x_{Variable{"x0"}, Variable{"x1"}};
  const Vector2<Variable> u_{Variable{"u0"}, Variable{"u1"}};
  const Vector2<Variable> p_{Variable{"p0"}, Variable{"p1"}};

  const Variable tc_{"tc"};
  const Vector2<Expression> xc_{Variable{"xc0"}, Variable{"xc1"}};
  const Vector2<Expression> uc_{Variable{"uc0"}, Variable{"uc1"}};
  const Vector2<Expression> pc_{Variable{"pc0"}, Variable{"pc1"}};
};

TEST_F(SymbolicVectorSystemTest, BuilderBasicPattern) {
  EXPECT_TRUE(SymbolicVectorSystemBuilder().time(t_).time()->equal_to(t_));

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().state(x_[1]).state()[0].equal_to(x_[1]));

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().state(x_).state()[1].equal_to(x_[1]));

  EXPECT_EQ(SymbolicVectorSystemBuilder().state({x_[0], x_[1]}).state(), x_);

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().input(u_[1]).input()[0].equal_to(u_[1]));

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().input(u_).input()[1].equal_to(u_[1]));

  EXPECT_EQ(SymbolicVectorSystemBuilder().input({u_[0], u_[1]}).input(), u_);

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().parameter(p_[1]).parameter()[0].equal_to(
          p_[1]));

  EXPECT_TRUE(
      SymbolicVectorSystemBuilder().parameter(p_).parameter()[1].equal_to(
          p_[1]));

  EXPECT_EQ(SymbolicVectorSystemBuilder().parameter({p_[0], p_[1]}).parameter(),
            p_);

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .dynamics(x_[0] + u_[0] + p_[0])
                  .dynamics()[0]
                  .EqualTo(x_[0] + u_[0] + p_[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .dynamics(Vector1<Expression>{x_[0] + u_[0] + p_[0]})
                  .dynamics()[0]
                  .EqualTo(x_[0] + u_[0] + p_[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .output(x_[0] + u_[0] + p_[0])
                  .output()[0]
                  .EqualTo(x_[0] + u_[0] + p_[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .output(Vector1<Expression>{x_[0] + u_[0] + p_[0]})
                  .output()[0]
                  .EqualTo(x_[0] + u_[0] + p_[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .output({x_[0] + u_[0] + p_[0], x_[1]})
                  .output()[0]
                  .EqualTo(x_[0] + u_[0] + p_[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .output({x_[0] + u_[0] + p_[0], x_[1]})
                  .output()[1]
                  .EqualTo(x_[1]));

  EXPECT_EQ(SymbolicVectorSystemBuilder().time_period(3.2).time_period(), 3.2);

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .state(x_[0])
                  .dynamics(-x_[0] + pow(x_[0], 3))
                  .state()[0]
                  .equal_to(x_[0]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .state(x_[0])
                  .dynamics(-x_[0] + pow(x_[0], 3))
                  .dynamics_for_variable(x_[0])
                  .EqualTo(-x_[0] + pow(x_[0], 3)));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .state(x_)
                  .dynamics(Vector2<Expression>(-x_[1], -x_[0] + 3))
                  .dynamics_for_variable(x_[1])
                  .EqualTo(-x_[0] + 3));

  EXPECT_THROW(SymbolicVectorSystemBuilder()
                   .state(x_)
                   .dynamics(Vector2<Expression>(-x_[1], -x_[0] + 3))
                   .dynamics_for_variable(u_[0]),
               std::out_of_range);

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .state(x_)
                  .dynamics({-x_[1], -x_[0] + 3})
                  .dynamics_for_variable(x_[0])
                  .EqualTo(-x_[1]));

  EXPECT_TRUE(SymbolicVectorSystemBuilder()
                  .state(x_)
                  .dynamics({-x_[1], -x_[0] + 3})
                  .dynamics_for_variable(x_[1])
                  .EqualTo(-x_[0] + 3));
}

// This test will fail on valgrind if we have dangling references.
TEST_F(SymbolicVectorSystemTest, DanglingReferenceTest) {
  SymbolicVectorSystemBuilder builder1;
  builder1.state(x_);
  EXPECT_TRUE(builder1.state()[0].equal_to(x_[0]));

  const auto& builder2 = SymbolicVectorSystemBuilder().state(x_);
  EXPECT_TRUE(builder2.state()[0].equal_to(x_[0]));
}

TEST_F(SymbolicVectorSystemTest, CubicPolyViaBuilder) {
  // xdot = -x + x^3 + p
  // y = x * p
  const Variable& x{x_[0]};
  const Variable& p{p_[0]};
  auto system = SymbolicVectorSystemBuilder()
                    .state(x)
                    .parameter(p)
                    .dynamics(-x + pow(x, 3) + p)
                    .output(x * p)
                    .Build();

  EXPECT_FALSE(system->time_var());

  ASSERT_EQ(system->state_vars().size(), 1);
  EXPECT_EQ(system->state_vars()[0], x);

  EXPECT_EQ(system->input_vars().size(), 0);

  ASSERT_EQ(system->parameter_vars().size(), 1);
  EXPECT_EQ(system->parameter_vars()[0], p);

  ASSERT_EQ(system->dynamics().size(), 1);
  EXPECT_PRED2(ExprEqual, system->dynamics()[0], -x + pow(x, 3) + p);
  EXPECT_PRED2(ExprEqual, system->dynamics_for_variable(x), -x + pow(x, 3) + p);

  ASSERT_EQ(system->output().size(), 1);
  EXPECT_EQ(system->output()[0], x * p);

  auto context = system->CreateDefaultContext();
  EXPECT_TRUE(context->has_only_continuous_state());
  EXPECT_EQ(context->num_total_states(), 1);
  EXPECT_EQ(system->num_input_ports(), 0);
  EXPECT_EQ(system->num_output_ports(), 1);
  EXPECT_EQ(system->num_numeric_parameter_groups(), 1);

  const double xval = 0.45;
  const double pval = 2.5;
  context->SetContinuousState(Vector1d{xval});
  context->get_mutable_numeric_parameter(0).SetAtIndex(0, pval);
  const auto& xdotval = system->EvalTimeDerivatives(*context);
  EXPECT_TRUE(CompareMatrices(xdotval.CopyToVector(),
                              Vector1d{-xval + xval * xval * xval + pval},
                              1e-14));
  EXPECT_TRUE(CompareMatrices(system->get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector1d{xval * pval}));
}

TEST_F(SymbolicVectorSystemTest, ScalarPassThrough) {
  // y = u.
  const Variable& u(u_[0]);
  SymbolicVectorSystem<double> system(
      {}, Vector0<Variable>{}, Vector1<Variable>{u}, Vector0<Expression>{},
      Vector1<Expression>{u});

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->is_stateless());
  EXPECT_EQ(system.get_input_port().size(), 1);
  EXPECT_EQ(system.get_output_port().size(), 1);
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 0));

  system.get_input_port().FixValue(context.get(), 0.12);
  EXPECT_TRUE(CompareMatrices(system.get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector1d{0.12}));
}

// A simple class whose sole purpose it to report when it's output gets
// calculated. This is a one-shot system. Once the output has been calculated,
// it will always report "calculated". Throw it out and create a new one as
// necessary.
// Trying to make it reusable requires coordinating the hidden mutable state in
// the system with the context -- the cache entry associated with the output
// would have to be marked "out of date". For this testing context, it's simpler
// to use it, evaluate it, and then throw it out.
template <typename T>
class CalcRecorder final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CalcRecorder);

  explicit CalcRecorder(int output_size = 1) {
    this->DeclareVectorOutputPort(kUseDefaultName, output_size,
                                  &CalcRecorder::CalcOutput);
  }

  bool calculated() const { return calculated_; }

 private:
  void CalcOutput(const Context<T>& context,
                  BasicVector<T>* output_vector) const {
    calculated_ = true;
    for (int i = 0; i < output_vector->size(); ++i) {
      output_vector->SetAtIndex(i, T(i * 0.5));
    }
  }

  mutable bool calculated_{false};
};

// Simply confirm the utility class does what it's supposed to do.
GTEST_TEST(SymbolicVectorSystemTestUtil, CalcRecorder) {
  CalcRecorder<double> recorder;
  ASSERT_FALSE(recorder.calculated());
  auto context = recorder.AllocateContext();
  recorder.get_output_port(0).Eval(*context);
  ASSERT_TRUE(recorder.calculated());
}

template <typename T>
class InputEvaluationTest : public ::testing::Test {};

// We want to confirm that the inputs are only evaluated for when required
// for *every* scalar type -- AutoDiffXd takes a different path than the other
// scalars.
using InputEvalTypes = ::testing::Types<double, AutoDiffXd, Expression>;
TYPED_TEST_SUITE(InputEvaluationTest, InputEvalTypes);

// This confirms that the input is only evaluated when it is *explicitly*
// required.
TYPED_TEST(InputEvaluationTest, EvaluateOnlyWhenNecessary) {
  using T = TypeParam;
  Variable u0("u0");
  Variable x0("x0");
  auto input_evaluated_for_output = [u0,
                                     x0](const Vector1<Expression>& output) {
    DiagramBuilder<T> builder;
    auto& recorder = *builder.template AddSystem<CalcRecorder<T>>();
    Vector1<Expression> dynamics{x0};
    auto& dut = *builder.template AddSystem<SymbolicVectorSystem<T>>(
        std::nullopt, Vector1<Variable>{x0}, Vector1<Variable>{u0}, dynamics,
        output);

    builder.Connect(recorder, dut);

    auto diagram = builder.Build();

    auto context = diagram->AllocateContext();
    const auto& dut_context = diagram->GetSubsystemContext(dut, *context);
    dut.get_output_port().Eval(dut_context);
    return recorder.calculated();
  };

  // Case: output simply doesn't reference the input.
  {
    Vector1<Expression> output{x0};
    ASSERT_FALSE(input_evaluated_for_output(output));
  }

  // Case: output references input.
  {
    Vector1<Expression> output{x0 + u0};
    ASSERT_TRUE(input_evaluated_for_output(output));
  }

  // Case: output references input in mathematically meaningless way.
  {
    Vector1<Expression> output{x0 + u0 * 0};
    ASSERT_FALSE(input_evaluated_for_output(output));
  }
}

TEST_F(SymbolicVectorSystemTest, VectorPassThrough) {
  // y = u.
  SymbolicVectorSystem<double> system({}, Vector0<Variable>{}, u_,
                                      Vector0<Expression>{},
                                      u_.cast<Expression>());

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->is_stateless());
  EXPECT_EQ(system.get_input_port().size(), 2);
  EXPECT_EQ(system.get_output_port().size(), 2);
  EXPECT_TRUE(system.HasDirectFeedthrough(0, 0));

  system.get_input_port().FixValue(context.get(), Vector2d{0.12, 0.34});
  EXPECT_TRUE(CompareMatrices(system.get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector2d{0.12, 0.34}));
}

TEST_F(SymbolicVectorSystemTest, OutputScaledTime) {
  SymbolicVectorSystem<double> system(
      t_, Vector0<Variable>{}, Vector0<Variable>{}, Vector0<Expression>{},
      Vector1<Expression>{2. * t_});

  ASSERT_TRUE(system.time_var());
  EXPECT_TRUE(system.time_var()->equal_to(t_));

  EXPECT_EQ(system.state_vars().size(), 0);

  EXPECT_EQ(system.input_vars().size(), 0);

  EXPECT_EQ(system.parameter_vars().size(), 0);

  EXPECT_EQ(system.dynamics().size(), 0);

  ASSERT_EQ(system.output().size(), 1);
  EXPECT_EQ(system.output()[0], 2.0 * t_);

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->is_stateless());
  EXPECT_EQ(system.num_input_ports(), 0);
  EXPECT_EQ(system.get_output_port().size(), 1);

  context->SetTime(2.0);
  EXPECT_TRUE(CompareMatrices(system.get_output_port()
                                  .template Eval<BasicVector<double>>(*context)
                                  .CopyToVector(),
                              Vector1d{4.0}));
}

TEST_F(SymbolicVectorSystemTest, ContinuousStateOnly) {
  // xdot = -x + x^3
  const Variable& x{x_[0]};
  SymbolicVectorSystem<double> system(
      {}, Vector1<Variable>{x}, Vector0<Variable>{},
      Vector1<Expression>{-x + pow(x, 3)}, Vector0<Expression>{});

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->has_only_continuous_state());
  EXPECT_EQ(context->num_total_states(), 1);
  EXPECT_EQ(system.num_input_ports(), 0);
  EXPECT_EQ(system.num_output_ports(), 0);

  const double xval = 0.45;
  context->SetContinuousState(Vector1d{xval});
  const auto& xdotval = system.EvalTimeDerivatives(*context);
  EXPECT_TRUE(CompareMatrices(xdotval.CopyToVector(),
                              Vector1d{-xval + xval * xval * xval}));
}

TEST_F(SymbolicVectorSystemTest, DiscreteStateOnly) {
  // xnext = -x + x^3
  const Variable& x{x_[0]};
  SymbolicVectorSystem<double> system(
      {}, Vector1<Variable>{x}, Vector0<Variable>{},
      Vector1<Expression>{-x + pow(x, 3)}, Vector0<Expression>{}, 0.1);

  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->has_only_discrete_state());
  EXPECT_EQ(context->num_total_states(), 1);
  EXPECT_EQ(system.num_input_ports(), 0);
  EXPECT_EQ(system.num_output_ports(), 0);

  const double xval = 0.45;
  context->get_mutable_discrete_state_vector()[0] = xval;
  auto discrete_variables = system.AllocateDiscreteVariables();
  system.CalcDiscreteVariableUpdates(*context, discrete_variables.get());
  EXPECT_TRUE(CompareMatrices(discrete_variables->get_vector().get_value(),
                              Vector1d{-xval + xval * xval * xval}));
}

TEST_F(SymbolicVectorSystemTest, IntegratorNoFeedthrough) {
  const Variable& x{x_[0]};
  const Variable& u{u_[0]};
  auto system = SymbolicVectorSystemBuilder()
                    .state(Vector1<Variable>{x})
                    .input(Vector1<Variable>{u})
                    .dynamics(Vector1<Expression>{u})
                    .output(Vector1<Expression>{x})
                    .Build<Expression>();

  EXPECT_FALSE(system->HasDirectFeedthrough(0, 0));
}

TEST_F(SymbolicVectorSystemTest, ContinuousTimeSymbolic) {
  auto system = SymbolicVectorSystemBuilder()
                    .time(t_)
                    .state(x_)
                    .input(u_)
                    .parameter(p_)
                    .dynamics(Vector2<Expression>{t_, x_[1] + u_[1] + p_[1]})
                    .output(Vector2<Expression>{x_[0] + u_[0] + p_[0], t_})
                    .Build<Expression>();

  ASSERT_TRUE(system->time_var());
  EXPECT_TRUE(system->time_var()->equal_to(t_));

  EXPECT_EQ(system->state_vars(), x_);

  EXPECT_EQ(system->input_vars(), u_);

  EXPECT_EQ(system->parameter_vars(), p_);

  ASSERT_EQ(system->dynamics().size(), 2);
  EXPECT_PRED2(ExprEqual, system->dynamics()[0], t_);
  EXPECT_PRED2(ExprEqual, system->dynamics_for_variable(x_[0]), t_);
  EXPECT_PRED2(ExprEqual, system->dynamics()[1], x_[1] + u_[1] + p_[1]);
  EXPECT_PRED2(ExprEqual, system->dynamics_for_variable(x_[1]),
               x_[1] + u_[1] + p_[1]);

  ASSERT_EQ(system->output().size(), 2);
  EXPECT_PRED2(ExprEqual, system->output()[0], x_[0] + u_[0] + p_[0]);
  EXPECT_PRED2(ExprEqual, system->output()[1], t_);

  auto context = system->CreateDefaultContext();

  context->SetTime(tc_);
  context->SetContinuousState(xc_);
  system->get_input_port().FixValue(context.get(), uc_);
  context->get_mutable_numeric_parameter(0).SetFromVector(pc_);

  const auto& xdot = system->EvalTimeDerivatives(*context).get_vector();
  EXPECT_TRUE(xdot.GetAtIndex(0).EqualTo(tc_));
  EXPECT_TRUE(xdot.GetAtIndex(1).EqualTo(xc_[1] + uc_[1] + pc_[1]));

  const auto& y = system->get_output_port()
                      .template Eval<BasicVector<Expression>>(*context)
                      .get_value();
  EXPECT_TRUE(y[0].EqualTo(xc_[0] + uc_[0] + pc_[0]));
  EXPECT_TRUE(y[1].EqualTo(tc_));
}

TEST_F(SymbolicVectorSystemTest, LinearizeDynamics) {
  // Create a system builder for the linearized pendulum model. Note that we
  // provide a symbolic/parametric linearization point (x_lp, u_lp).

  // state
  const Variable theta{"theta"};
  const Variable theta_dot{"theta_dot"};
  const Vector2<Variable> x{theta, theta_dot};

  // input
  const Variable tau{"tau"};
  const Vector1<Variable> u{tau};

  // parameter
  const Variable m{"m"};  // mass
  const Variable g{"g"};  // gravity
  const Variable l{"l"};  // length
  const Variable b{"b"};  // damping
  const Vector4<Variable> p{m, g, l, b};

  // non-linear dynamics
  const Vector2<Expression> dynamics{
      theta_dot, (tau - m * g * l * sin(theta) - b * theta_dot) / (m * l * l)};

  // Linearization Point
  const Variable theta_lp{"theta_lp"};
  const Variable theta_dot_lp{"theta_dot_lp"};
  const Variable tau_lp{"tau_lp"};
  const Vector2<Expression> x_lp{theta_lp, theta_dot_lp};
  const Vector1<Expression> u_lp{tau_lp};

  auto system_builder = SymbolicVectorSystemBuilder()
                            .time(t_)
                            .state(x)
                            .input(u)
                            .parameter(p)
                            .dynamics(dynamics)
                            .output(Vector2<Expression>{theta, theta_dot})
                            .LinearizeDynamics(x_lp, u_lp);

  // \dot{theta} = theta_dot which was already linear.
  EXPECT_PRED2(ExprEqual, system_builder.dynamics_for_variable(theta),
               theta_dot);

  // x_lp and u_lp were not declared as system parameters. But the
  // `LinearlizeDynamics` method introduces them as parameters.
  const Variables new_parameters{system_builder.parameter()};
  EXPECT_EQ(new_parameters.size(),
            p.size() + 3 /* dim of the linearization point */);
  EXPECT_TRUE(new_parameters.include(theta_lp));
  EXPECT_TRUE(new_parameters.include(theta_dot_lp));
  EXPECT_TRUE(new_parameters.include(tau_lp));

  // Check that the linearized dynamics for theta_dot is linear in state and
  // input ({theta, theta_dot, tau}).
  const Polynomial linearized_dynamics_for_theta_dot{
      system_builder.dynamics_for_variable(theta_dot),
      {theta, theta_dot, tau} /* indeterminates */
  };
  EXPECT_EQ(linearized_dynamics_for_theta_dot.TotalDegree(), 1 /* linear */);

  // Pick two sample linearization points and evaluate the
  // `linearized_dynamics_for_theta_dot` polynomial. Make sure that the
  // evaluation results are matched with the linearization results using
  // `TaylorExpand` (outside of SymbolicVectorSystemBuilder class).
  {
    const Environment env1{
        {theta_lp, -0.1}, {theta_dot_lp, 0.2}, {tau_lp, 0.3}};
    const Environment env2{{theta, env1[theta_lp]},
                           {theta_dot, env1[theta_dot_lp]},
                           {tau, env1[tau_lp]}};

    const Polynomial p1{
        linearized_dynamics_for_theta_dot.EvaluatePartial(env1)};
    const Polynomial p2{TaylorExpand(dynamics[1], env2, 1 /* linear */),
                        {theta, theta_dot, tau} /* indeterminates */};

    EXPECT_PRED2(PolyEqual, p1, p2);
  }

  {
    const Environment env1{
        {theta_lp, 0.3}, {theta_dot_lp, -0.1}, {tau_lp, 0.4}};
    const Environment env2{{theta, env1[theta_lp]},
                           {theta_dot, env1[theta_dot_lp]},
                           {tau, env1[tau_lp]}};

    const Polynomial p1{
        linearized_dynamics_for_theta_dot.EvaluatePartial(env1)};
    const Polynomial p2{TaylorExpand(dynamics[1], env2, 1 /* linear */),
                        {theta, theta_dot, tau} /* indeterminates */};

    EXPECT_PRED2(PolyEqual, p1, p2);
  }
}

TEST_F(SymbolicVectorSystemTest, DiscreteTimeSymbolic) {
  auto system = SymbolicVectorSystemBuilder()
                    .time(t_)
                    .state(x_)
                    .input(u_)
                    .parameter(p_)
                    .dynamics(Vector2<Expression>{t_, x_[1] + u_[1] + p_[1]})
                    .output(Vector2<Expression>{x_[0] + u_[0] + p_[0], t_})
                    .time_period(1.0)
                    .Build<Expression>();

  ASSERT_TRUE(system->time_var());
  EXPECT_TRUE(system->time_var()->equal_to(t_));

  EXPECT_EQ(system->state_vars(), x_);

  EXPECT_EQ(system->input_vars(), u_);

  EXPECT_EQ(system->parameter_vars(), p_);

  ASSERT_EQ(system->dynamics().size(), 2);
  EXPECT_PRED2(ExprEqual, system->dynamics()[0], t_);
  EXPECT_PRED2(ExprEqual, system->dynamics_for_variable(x_[0]), t_);
  EXPECT_PRED2(ExprEqual, system->dynamics()[1], x_[1] + u_[1] + p_[1]);
  EXPECT_PRED2(ExprEqual, system->dynamics_for_variable(x_[1]),
               x_[1] + u_[1] + p_[1]);

  ASSERT_EQ(system->output().size(), 2);
  EXPECT_PRED2(ExprEqual, system->output()[0], x_[0] + u_[0] + p_[0]);
  EXPECT_PRED2(ExprEqual, system->output()[1], t_);

  auto context = system->CreateDefaultContext();

  context->SetTime(tc_);
  context->get_mutable_discrete_state_vector().SetFromVector(xc_);
  system->get_input_port().FixValue(context.get(), uc_);
  context->get_mutable_numeric_parameter(0).SetFromVector(pc_);

  auto discrete_variables = system->AllocateDiscreteVariables();
  system->CalcDiscreteVariableUpdates(*context, discrete_variables.get());
  const auto& xnext = discrete_variables->get_vector().get_value();
  EXPECT_TRUE(xnext[0].EqualTo(tc_));
  EXPECT_TRUE(xnext[1].EqualTo(xc_[1] + uc_[1] + pc_[1]));

  const auto& y = system->get_output_port()
                      .template Eval<BasicVector<Expression>>(*context)
                      .get_value();
  EXPECT_TRUE(y[0].EqualTo(xc_[0] + uc_[0] + pc_[0]));
  EXPECT_TRUE(y[1].EqualTo(tc_));
}

TEST_F(SymbolicVectorSystemTest, TestScalarConversion) {
  const Variable& x{x_[0]};
  const Variable& p{p_[0]};
  auto system = SymbolicVectorSystemBuilder()
                    .state(x)
                    .parameter(p)
                    .dynamics(-x + p)
                    .Build();

  EXPECT_TRUE(is_autodiffxd_convertible(*system));
  EXPECT_TRUE(is_symbolic_convertible(*system));
}

class SymbolicVectorSystemAutoDiffXdTest : public SymbolicVectorSystemTest {
 protected:
  void SetUp() override {
    txupval_ << tval_, xval_, uval_, pval_;

    system_ = SymbolicVectorSystemBuilder()
                  .time(t_)
                  .state(x_[0])
                  .input(u_)
                  .parameter(p_)
                  .dynamics(-x_[0] * p_[0] + pow(x_[0], 3))
                  .output(t_ + 2. * u_[0] + 3. * u_[1] + p_[1])
                  .Build();

    // clang-format off
    expected_xdotval0_deriv_ << 0.0,                             // ∂/∂t
                               -pval_[0] + 3.0 * xval_ * xval_,  // ∂/∂x
                                0.0,                             // ∂/∂u₀
                                0.0,                             // ∂/∂u₁
                               -xval_,                           // ∂/∂p₀
                                0.0;                             // ∂/∂p₁

    expected_yval0_deriv_ << 1.0,                               // ∂/∂t
                             0.0,                               // ∂/∂x
                             2.0,                               // ∂/∂u₀
                             3.0,                               // ∂/∂u1
                             0.0,                               // ∂/∂p₀
                             1.0;                               // ∂/∂p1
    // clang-format on

    autodiff_system_ = system_->ToAutoDiffXd();
    context_ = autodiff_system_->CreateDefaultContext();
  }

  void SetContext(const VectorX<AutoDiffXd>& txup) {
    context_->SetTime(txup[0]);
    context_->SetContinuousState(txup.segment<1>(1));
    autodiff_system_->get_input_port(0).FixValue(context_.get(),
                                                 txup.segment<2>(2));
    context_->get_mutable_numeric_parameter(0).SetFromVector(txup.tail<2>());
  }

  const double tval_{1.23};
  const double xval_{0.45};
  const Vector2d uval_{5.6, 7.8};
  const Vector2d pval_{2.0, 12.0};
  Vector6d txupval_;
  Vector6d expected_xdotval0_deriv_;
  Vector6d expected_yval0_deriv_;

  std::unique_ptr<SymbolicVectorSystem<double>> system_;
  std::unique_ptr<System<AutoDiffXd>> autodiff_system_;
  std::unique_ptr<Context<AutoDiffXd>> context_;
};

TEST_F(SymbolicVectorSystemAutoDiffXdTest, AutodiffXdFullGradient) {
  const VectorX<AutoDiffXd> txup = math::InitializeAutoDiff(txupval_);
  SetContext(txup);

  const auto xdotval =
      autodiff_system_->EvalTimeDerivatives(*context_).CopyToVector();
  EXPECT_TRUE(
      CompareMatrices(math::ExtractValue(xdotval),
                      Vector1d{-xval_ * pval_[0] + xval_ * xval_ * xval_}));
  EXPECT_TRUE(
      CompareMatrices(xdotval[0].derivatives(), expected_xdotval0_deriv_));

  const auto& yval = autodiff_system_->get_output_port(0)
                         .template Eval<BasicVector<AutoDiffXd>>(*context_)
                         .get_value();
  EXPECT_TRUE(CompareMatrices(
      math::ExtractValue(yval),
      Vector1d{tval_ + 2 * uval_[0] + 3 * uval_[1] + pval_[1]}));
  EXPECT_TRUE(CompareMatrices(yval[0].derivatives(), expected_yval0_deriv_));
}

TEST_F(SymbolicVectorSystemAutoDiffXdTest, AutodiffXdGradientRelativeToInput) {
  Eigen::MatrixXd gradient{Eigen::MatrixXd::Zero(6, 2)};
  gradient(2, 0) = 1.0;  // u₀
  gradient(3, 1) = 1.0;  // u₁
  const VectorX<AutoDiffXd> txup =
      math::InitializeAutoDiff(Eigen::MatrixXd(txupval_), gradient);
  SetContext(txup);

  const auto xdotval =
      autodiff_system_->EvalTimeDerivatives(*context_).CopyToVector();
  EXPECT_TRUE(CompareMatrices(xdotval[0].derivatives(),
                              expected_xdotval0_deriv_.segment<2>(2)));

  const auto& yval = autodiff_system_->get_output_port(0)
                         .template Eval<BasicVector<AutoDiffXd>>(*context_)
                         .get_value();
  EXPECT_TRUE(CompareMatrices(yval[0].derivatives(),
                              expected_yval0_deriv_.segment<2>(2)));
}

// Similar to AutodiffXdGradientRelativeToInput, but this time with some empty
// derivatives.
TEST_F(SymbolicVectorSystemAutoDiffXdTest, AutodiffXdGradientRelativeToState) {
  VectorX<AutoDiffXd> txup = txupval_;
  txup.head<2>() = math::InitializeAutoDiff(txupval_.head<2>());
  SetContext(txup);

  const auto xdotval =
      autodiff_system_->EvalTimeDerivatives(*context_).CopyToVector();
  EXPECT_TRUE(CompareMatrices(xdotval[0].derivatives(),
                              expected_xdotval0_deriv_.head<2>()));

  const auto& yval = autodiff_system_->get_output_port(0)
                         .template Eval<BasicVector<AutoDiffXd>>(*context_)
                         .get_value();
  EXPECT_TRUE(CompareMatrices(yval[0].derivatives(),
                              expected_yval0_deriv_.head<2>()));
}

TEST_F(SymbolicVectorSystemAutoDiffXdTest,
       AutodiffXdGradientRelativeToParameter) {
  Eigen::MatrixXd gradient{Eigen::MatrixXd::Zero(6, 2)};
  gradient(4, 0) = 1.0;  // p₀
  gradient(5, 1) = 1.0;  // p₁
  const VectorX<AutoDiffXd> txup =
      math::InitializeAutoDiff(Eigen::MatrixXd(txupval_), gradient);
  SetContext(txup);

  const auto xdotval =
      autodiff_system_->EvalTimeDerivatives(*context_).CopyToVector();
  EXPECT_TRUE(CompareMatrices(xdotval[0].derivatives(),
                              expected_xdotval0_deriv_.segment<2>(4)));

  const auto& yval = autodiff_system_->get_output_port(0)
                         .template Eval<BasicVector<AutoDiffXd>>(*context_)
                         .get_value();
  EXPECT_TRUE(CompareMatrices(yval[0].derivatives(),
                              expected_yval0_deriv_.segment<2>(4)));
}

}  // namespace
}  // namespace systems
}  // namespace drake
