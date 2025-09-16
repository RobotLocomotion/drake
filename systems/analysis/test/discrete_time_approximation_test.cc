#include "drake/systems/analysis/discrete_time_approximation.h"

#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(DiscreteTimeApproxTest, AffineAndLinear) {
  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Vector<double, 2> f0;
  Eigen::Matrix<double, 1, 2> C;
  Eigen::Matrix<double, 1, 1> D;
  Eigen::Vector<double, 1> y0;
  A << 0, 1, 0, 0;
  B << 0, 1;
  f0 << 2, 1;
  C << 1, 0;
  D << 1;
  y0 << 1;

  // Reject discrete system as argument.
  EXPECT_ANY_THROW(DiscreteTimeApproximation<double>(
      AffineSystem<double>(A, B, f0, C, D, y0, 0.1), 0.1));

  const double h = 0.031415926;

  Eigen::Matrix<double, 2, 2> Ad;
  Eigen::Matrix<double, 2, 1> Bd;
  Eigen::Vector<double, 2> f0d;
  Eigen::Matrix<double, 1, 2> Cd = C;
  Eigen::Matrix<double, 1, 1> Dd = D;
  Eigen::Vector<double, 1> y0d = y0;
  Ad << 1, h, 0, 1;
  Bd << 0.5 * h * h, h;
  f0d << 2 * h + 0.5 * h * h, h;

  AffineSystem<double> continuous_affine_sys(A, B, f0, C, D, y0);
  auto discrete_affine_sys =
      DiscreteTimeApproximation<double>(continuous_affine_sys, h);

  const double kCompTolerance = 1e-14;
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->A(), Ad, kCompTolerance));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->B(), Bd, kCompTolerance));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->f0(), f0d, kCompTolerance));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->C(), Cd));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->D(), Dd));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->y0(), y0d));

  LinearSystem<double> continuous_linear_sys(A, B, C, D);
  auto discrete_linear_sys =
      DiscreteTimeApproximation<double>(continuous_affine_sys, h);

  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->A(), Ad, kCompTolerance));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->B(), Bd, kCompTolerance));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->C(), Cd));
  EXPECT_TRUE(CompareMatrices(discrete_affine_sys->D(), Dd));
}

GTEST_TEST(DiscreteTimeApproxTest, IntegrateConstant) {
  DiagramBuilder<double> builder;

  const double time_period = 0.125;
  Integrator<double> integrator_sys(1);
  auto dut =
      builder.AddSystem(DiscreteTimeApproximation(integrator_sys, time_period));
  // Connect a constant 1.0 to the discrete-time approximation of the
  // integrator
  auto source = builder.AddSystem<ConstantVectorSource>(
      Eigen::Matrix<double, 1, 1>::Ones());
  builder.Connect(source->get_output_port(), dut->get_input_port());
  auto logger =
      builder.AddSystem<VectorLogSink>(1, /* log period = */ 0.015625);
  builder.Connect(dut->get_output_port(), logger->get_input_port());

  auto diagram = builder.Build();

  Simulator simulator(*diagram);
  const double tf = 1.001;
  simulator.AdvanceTo(tf);

  auto& log = logger->FindLog(simulator.get_context());
  for (int i = 0; i < log.num_samples(); ++i) {
    // expected = 0    if t = 0
    // expected = Δt   ∀t ∈ (0,Δt]
    // expected = 2Δt  ∀t ∈ (Δt,2Δt] ...
    const double t = log.sample_times()[i];
    const double expected = std::ceil(t / time_period) * time_period;
    ASSERT_NEAR(log.data().coeff(i), expected, 1e-14);
  }
}

GTEST_TEST(DiscreteTimeApproxTest, IntegrateRamp) {
  DiagramBuilder<double> builder;
  auto source = builder.AddSystem<ConstantVectorSource>(
      Eigen::Matrix<double, 1, 1>::Ones());
  auto ramp = builder.AddSystem<Integrator>(1);
  builder.Connect(source->get_output_port(), ramp->get_input_port());

  const double time_period = 0.01;
  Integrator<double> integrator_sys(1);
  auto sys =
      builder.AddSystem(DiscreteTimeApproximation(integrator_sys, time_period));
  // Connect a ramp of slope 1 to the discrete-time approximation of the
  // integrator. Since the discrete-time approximation has zero-order hold on
  // the input, the output is the area under a staircase.
  builder.Connect(ramp->get_output_port(), sys->get_input_port());
  builder.ExportOutput(sys->get_output_port());

  auto diagram = builder.Build();

  Simulator simulator(*diagram);
  const double tf = 1.001;
  simulator.AdvanceTo(tf);

  const int num_steps = std::ceil(tf / time_period);
  const double area =
      (num_steps - 1) * num_steps / 2 * (time_period * time_period);
  EXPECT_NEAR(area, diagram->get_output_port().Eval(simulator.get_context())[0],
              1e-12);
}

template <typename T>
class OutputTimeStateSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputTimeStateSystem);

  // This class is intended to be not scalar convertible.
  OutputTimeStateSystem() {
    ContinuousStateIndex state_index = this->DeclareContinuousState(
        BasicVector<T>(Eigen::VectorX<T>::Ones(1) * 3.14));
    this->DeclareVectorOutputPort(
        "t", 1, [](const Context<T>& context, BasicVector<T>* out) {
          out->get_mutable_value()[0] = context.get_time();
        });
    this->DeclareStateOutputPort("x", state_index);
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final {
    (*derivatives)[0] = 1.0;  // Slope is 1.0.
  }
};

GTEST_TEST(DiscreteTimeApproxTest, DiscreteUpdateShouldNotChangeTimeState) {
  OutputTimeStateSystem<double> output_time_system;
  const double time_period = 0.1;
  auto sys = DiscreteTimeApproximation(output_time_system, time_period);
  auto context = sys->CreateDefaultContext();
  EXPECT_TRUE(context->has_only_discrete_state() &&
              context->num_discrete_state_groups() == 1);
  EXPECT_EQ(context->get_discrete_state().value()[0], 3.14);

  double t = 5.0;
  Eigen::VectorXd x = Eigen::VectorXd::Ones(1);
  context->SetTime(t);
  context->SetDiscreteState(x);

  const DiscreteValues<double>& updated =
      sys->EvalUniquePeriodicDiscreteUpdate(*context);

  // The time output should remain the same as it was originally set.
  EXPECT_EQ(sys->get_output_port(0).Eval(*context)[0], t);

  // The discrete state hasn't yet changed, so the state output should remain
  // the same as it was originally set.
  EXPECT_EQ(sys->get_output_port(1).Eval(*context)[0], x[0]);

  // This finally changes the discrete state.
  context->SetDiscreteState(updated);
  EXPECT_NEAR(sys->get_output_port(1).Eval(*context)[0], x[0] + time_period,
              1e-14);
}

template <typename T>
class DiscreteTimeApproxScalarConversionTest : public ::testing::Test {};

using DefaultScalars =
    ::testing::Types<double, AutoDiffXd, symbolic::Expression>;
TYPED_TEST_SUITE(DiscreteTimeApproxScalarConversionTest, DefaultScalars);

TYPED_TEST(DiscreteTimeApproxScalarConversionTest, Convertible) {
  using T = TypeParam;
  Integrator<T> dummy_sys(1);
  SimulatorConfig config{.integration_scheme = "explicit_euler"};
  auto sys = DiscreteTimeApproximation(dummy_sys, 0.1, config);

  using TargetType =
      std::conditional_t<std::is_same_v<T, double>, AutoDiffXd, double>;
  DRAKE_EXPECT_NO_THROW(sys->template ToScalarType<TargetType>());

  DRAKE_EXPECT_NO_THROW(sys->Clone());
}

/* We minimally test that the discrete-time approximated system is not scalar
 convertible due to either
 1) the continuous-time system not scalar convertible or
 2) the integrator does not support the target scalar type.
 We do not explicit test the case where both reasons occur simultaneously. */
TYPED_TEST(DiscreteTimeApproxScalarConversionTest,
           ContinuousTimeSystemNotConvertible) {
  using T = TypeParam;
  OutputTimeStateSystem<T> dummy_sys;
  SimulatorConfig config{.integration_scheme = "explicit_euler"};
  auto sys = DiscreteTimeApproximation(dummy_sys, 0.1, config);

  using TargetType =
      std::conditional_t<std::is_same_v<T, double>, AutoDiffXd, double>;
  auto target_type_name = NiceTypeName::Get<TargetType>();

  DRAKE_EXPECT_THROWS_MESSAGE(
      sys->template ToScalarType<TargetType>(),
      fmt::format(".+ does not support scalar conversion to type {} \\(because "
                  ".+<{}> does not support scalar conversion to type {}\\)",
                  target_type_name, NiceTypeName::Get<T>(), target_type_name));

  auto converted =
      sys->template ToScalarTypeMaybe<TargetType>();  // Should not throw.
  EXPECT_EQ(converted, nullptr);

  DRAKE_EXPECT_THROWS_MESSAGE(sys->Clone(), ".+ does not support Cloning");
}

TYPED_TEST(DiscreteTimeApproxScalarConversionTest, IntegratorNotConvertible) {
  using T = TypeParam;
  Integrator<T> dummy_sys(1);
  SimulatorConfig config{.integration_scheme = "runge_kutta5"};

  if constexpr (!std::is_same_v<T, symbolic::Expression>) {
    auto sys = DiscreteTimeApproximation(dummy_sys, 0.1, config);

    auto target_type_name = NiceTypeName::Get<symbolic::Expression>();
    DRAKE_EXPECT_THROWS_MESSAGE(
        sys->ToSymbolic(),
        fmt::format(
            ".+ does not support scalar conversion to type {} \\(because the "
            "integration scheme .+ does not support scalar type {}\\)",
            target_type_name, target_type_name));

    auto converted = sys->ToSymbolicMaybe();  // Should not throw.
    EXPECT_EQ(converted, nullptr);

    DRAKE_EXPECT_NO_THROW(sys->Clone());
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        DiscreteTimeApproximation(dummy_sys, 0.1, config),
        "Integration scheme .+ does not support scalar type " +
            NiceTypeName::Get<symbolic::Expression>());
  }
}

template <typename T>
class DirectFeedthroughSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectFeedthroughSystem);

  DirectFeedthroughSystem(int num_input_ports, int num_output_ports,
                          const std::multimap<int, int>& direct_feedthroughs) {
    this->DeclareContinuousState(1);

    for (int i = 0; i < num_input_ports; ++i) {
      this->DeclareAbstractInputPort(kUseDefaultName,
                                     Value(Eigen::Matrix3<T>::Zero().eval()));
    }

    for (int j = 0; j < num_output_ports; ++j) {
      std::vector<int> connected_input_ports;
      for (const auto& [input_index, output_index] : direct_feedthroughs) {
        if (output_index == j) {
          connected_input_ports.push_back(input_index);
        }
      }

      std::set<DependencyTicket> prerequisites_of_calc{};
      for (int i : connected_input_ports) {
        prerequisites_of_calc.insert(
            this->input_port_ticket(InputPortIndex(i)));
      }
      if (prerequisites_of_calc.empty()) {
        prerequisites_of_calc.insert(SystemBase::nothing_ticket());
      }

      this->DeclareAbstractOutputPort(
          kUseDefaultName,
          []() {
            return std::make_unique<Value<Eigen::Matrix3<T>>>(
                Eigen::Matrix3<T>::Zero());
          },
          [this, connected_input_ports](const Context<T>& context,
                                        AbstractValue* out) {
            Eigen::Matrix3<T> value;
            value.setZero();
            for (int i : connected_input_ports) {
              value += this->get_input_port(i).template Eval<Eigen::Matrix3<T>>(
                  context);
            }
            out->set_value(value);
          },
          prerequisites_of_calc);
    }
  }

  void DoCalcTimeDerivatives(const Context<T>&,
                             ContinuousState<T>*) const final {
    DRAKE_UNREACHABLE();
  }
};

GTEST_TEST(DiscreteTimeApproxAbstractOutputTest, NoFeedthrough) {
  //       ┌─────────────┐
  //   ────┼ u0 ─   ─ y0 ┼────
  //       └─────────────┘
  DirectFeedthroughSystem<double> dummy_sys(1, 1, {});

  auto sys = DiscreteTimeApproximation(dummy_sys, 0.1);
  EXPECT_FALSE(sys->HasAnyDirectFeedthrough());

  auto context = sys->CreateDefaultContext();
  EXPECT_TRUE(
      CompareMatrices(sys->get_output_port().Eval<Eigen::Matrix3d>(*context),
                      Eigen::Matrix3d::Zero()));
}

GTEST_TEST(DiscreteTimeApproxAbstractOutputTest, SingleFeedthrough) {
  //       ┌─────────────┐
  //   ────┼ u0 ───── y0 ┼────
  //       └─────────────┘
  DirectFeedthroughSystem<double> dummy_sys(1, 1, {{0, 0}});

  auto sys = DiscreteTimeApproximation(dummy_sys, 0.1);
  EXPECT_TRUE(sys->HasAnyDirectFeedthrough());

  auto context = sys->CreateDefaultContext();
  Eigen::Matrix3d value = Eigen::Matrix3d::Identity();
  sys->get_input_port().FixValue(context.get(), Value(value));
  EXPECT_TRUE(CompareMatrices(
      sys->get_output_port().Eval<Eigen::Matrix3d>(*context), value));
}

GTEST_TEST(DiscreteTimeApproxAbstractOutputTest, DirectFeedback) {
  //       ┌──────────────┐
  //   ────┼ u0 ────── y0 ┼─┬──
  //     ┌─┼ u1 ─         │ │
  //     │ └──────────────┘ │
  //     └──────────────────┘
  DirectFeedthroughSystem<double> dummy_sys(2, 1, {{0, 0}});

  // If DiscreteTimeApproximation() is implemented incorrectly, such as calling
  // u1.Eval() within y0.Eval(), it will result in an infinite loop.
  auto discrete_sys = DiscreteTimeApproximation(dummy_sys, 0.1);

  DiagramBuilder<double> builder;
  auto sys = builder.AddSystem(std::move(discrete_sys));
  builder.Connect(sys->get_output_port(0), sys->get_input_port(1));
  builder.ExportInput(sys->get_input_port(0));
  builder.ExportOutput(sys->get_output_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  Eigen::Matrix3d value = Eigen::Matrix3d::Identity();
  diagram->get_input_port().FixValue(context.get(), Value(value));
  EXPECT_TRUE(CompareMatrices(
      diagram->get_output_port().Eval<Eigen::Matrix3d>(*context), value));
}

GTEST_TEST(DiscreteTimeApproxAbstractOutputTest, MultipleDirectFeedback) {
  //       ┌──────────────┐
  //   ────┼ u0 ────── y0 ┼─┐
  //     ┌─┼ u1 ────── y1 ┼─┼─┐
  //   ┌─┼─┼ u2 ────── y2 ┼─┼─┼──
  //   │ │ └──────────────┘ │ │
  //   │ └──────────────────┘ │
  //   └──────────────────────┘
  DirectFeedthroughSystem<double> dummy_sys(3, 3, {{0, 0}, {1, 1}, {2, 2}});

  auto discrete_sys = DiscreteTimeApproximation(dummy_sys, 0.1);

  DiagramBuilder<double> builder;
  auto sys = builder.AddSystem(std::move(discrete_sys));
  builder.Connect(sys->get_output_port(0), sys->get_input_port(1));
  builder.Connect(sys->get_output_port(1), sys->get_input_port(2));
  builder.ExportInput(sys->get_input_port(0));
  builder.ExportOutput(sys->get_output_port(2));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  Eigen::Matrix3d value1 = Eigen::Matrix3d::Identity();
  diagram->get_input_port().FixValue(context.get(), Value(value1));
  EXPECT_TRUE(CompareMatrices(
      diagram->get_output_port().Eval<Eigen::Matrix3d>(*context), value1));

  Eigen::Matrix3d value2 = Eigen::Matrix3d::Ones();
  diagram->get_input_port().FixValue(context.get(), Value(value2));
  EXPECT_TRUE(CompareMatrices(
      diagram->get_output_port().Eval<Eigen::Matrix3d>(*context), value2));
}

class DiscreteTimeApproxMultibodyPlantTest : public ::testing::Test {
 public:
  void SetUp() override {
    using multibody::MultibodyPlant;

    continuous_sys_ = std::make_unique<MultibodyPlant<double>>(0.0);
    multibody::Parser(continuous_sys_.get())
        .AddModelsFromUrl(
            "package://drake/examples/multibody/cart_pole/cart_pole.sdf");
    continuous_sys_->Finalize();

    discrete_sys_ = DiscreteTimeApproximation(*continuous_sys_, time_period_);
  }

 protected:
  std::unique_ptr<AffineSystem<double>> DiscreteStabilizingController() const {
    Eigen::VectorXd xd(4);
    xd << 0, M_PI, 0, 0;
    Eigen::VectorXd ud(1);
    ud << 0;

    auto discrete_context = discrete_sys_->CreateDefaultContext();
    discrete_context->SetDiscreteState(xd);
    auto& input_port = discrete_sys_->GetInputPort("actuation");
    input_port.FixValue(discrete_context.get(), ud);

    Eigen::DiagonalMatrix<double, 4> Q;
    Q.diagonal() << 10, 10, 1, 1;
    Eigen::DiagonalMatrix<double, 1> R;
    R.diagonal() << 0.01;
    Eigen::Matrix<double, 0, 0> N;

    return controllers::LinearQuadraticRegulator(
        *discrete_sys_, *discrete_context, Eigen::MatrixXd(Q),
        Eigen::MatrixXd(R), N, input_port.get_index());
  }

  std::unique_ptr<multibody::MultibodyPlant<double>> continuous_sys_;
  std::unique_ptr<System<double>> discrete_sys_;
  const double time_period_ = 0.01;
};

TEST_F(DiscreteTimeApproxMultibodyPlantTest, Construction) {
  auto continuous_context = continuous_sys_->CreateDefaultContext();
  auto discrete_context = discrete_sys_->CreateDefaultContext();

  EXPECT_TRUE(continuous_context->has_only_continuous_state());
  EXPECT_TRUE(discrete_context->has_only_discrete_state() &&
              discrete_context->num_discrete_state_groups() == 1);

  EXPECT_EQ(continuous_context->num_continuous_states(),
            discrete_context->get_discrete_state_vector().size());
  EXPECT_TRUE(
      CompareMatrices(continuous_context->get_continuous_state().CopyToVector(),
                      discrete_context->get_discrete_state().value()));
}

TEST_F(DiscreteTimeApproxMultibodyPlantTest, InputPortsMatch) {
  EXPECT_EQ(continuous_sys_->num_input_ports(),
            discrete_sys_->num_input_ports());
  for (int i = 0; i < discrete_sys_->num_input_ports(); ++i) {
    auto& port1 = continuous_sys_->get_input_port(i);
    auto& port2 = discrete_sys_->get_input_port(i);
    EXPECT_EQ(port1.get_name(), port2.get_name());
    EXPECT_EQ(port1.get_data_type(), port2.get_data_type());
    EXPECT_EQ(port1.size(), port2.size());
    EXPECT_EQ(port1.get_random_type(), port2.get_random_type());
  }
}

TEST_F(DiscreteTimeApproxMultibodyPlantTest, OutputPortsMatch) {
  EXPECT_EQ(continuous_sys_->num_output_ports(),
            discrete_sys_->num_output_ports());
  for (int i = 0; i < discrete_sys_->num_output_ports(); ++i) {
    auto& port1 = continuous_sys_->get_output_port(i);
    auto& port2 = discrete_sys_->get_output_port(i);
    EXPECT_EQ(port1.get_name(), port2.get_name());
    EXPECT_EQ(port1.get_data_type(), port2.get_data_type());
    EXPECT_EQ(port1.size(), port2.size());
  }
}

TEST_F(DiscreteTimeApproxMultibodyPlantTest, DirectFeedthroughsMatch) {
  EXPECT_THAT(continuous_sys_->GetDirectFeedthroughs(),
              ::testing::UnorderedElementsAreArray(
                  discrete_sys_->GetDirectFeedthroughs()));
}

TEST_F(DiscreteTimeApproxMultibodyPlantTest, SynthesizeDiscreteTimeLQR) {
  DiagramBuilder<double> builder;
  auto plant = builder.AddSystem(std::move(continuous_sys_));
  auto controller = builder.AddSystem(DiscreteStabilizingController());
  auto zoh = builder.AddSystem<ZeroOrderHold>(time_period_, 1);
  builder.Connect(plant->get_state_output_port(), controller->get_input_port());
  builder.Connect(controller->get_output_port(), zoh->get_input_port());
  builder.Connect(zoh->get_output_port(), plant->get_actuation_input_port());

  auto diagram = builder.Build();
  Simulator simulator(*diagram);

  const double theta_init = M_PI * 0.9;
  Eigen::VectorXd x_init(4);
  x_init << 0, theta_init, 0, 0;
  simulator.get_mutable_context().SetContinuousState(x_init);

  simulator.AdvanceTo(1.0);
  const double theta_final =
      simulator.get_context().get_continuous_state_vector()[1];
  // Check discrete-time controller stabilizes continuous-time cart-pole.
  const double theta_d = M_PI;
  EXPECT_LT(std::abs(theta_final - theta_d), std::abs(theta_init - theta_d));
}

TEST_F(DiscreteTimeApproxMultibodyPlantTest, DiscretePlantSimulatable) {
  DiagramBuilder<double> builder;
  auto controller = builder.AddSystem(DiscreteStabilizingController());
  auto plant = builder.AddSystem(std::move(discrete_sys_));
  builder.Connect(plant->GetOutputPort("state"), controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  plant->GetInputPort("actuation"));

  auto diagram = builder.Build();
  Simulator simulator(*diagram);

  const double theta_init = M_PI * 0.9;
  Eigen::VectorXd x_init(4);
  x_init << 0, theta_init, 0, 0;
  simulator.get_mutable_context().SetDiscreteState(x_init);

  simulator.AdvanceTo(1.0);
  const double theta_final = simulator.get_context().get_discrete_state()[1];
  // Check discrete-time controller stabilizes discrete-time cart-pole.
  const double theta_d = M_PI;
  EXPECT_LT(std::abs(theta_final - theta_d), std::abs(theta_init - theta_d));
}

}  // namespace
}  // namespace systems
}  // namespace drake
