#include "drake/systems/analysis/discrete_time_approximation.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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

GTEST_TEST(DiscreteTimeApproximation, AffineSystemTest) {
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

GTEST_TEST(DiscreteTimeApproximation, IntegrateConstantTest) {
  DiagramBuilder<double> builder;
  auto source = builder.AddSystem<ConstantVectorSource>(
      Eigen::Matrix<double, 1, 1>::Ones());
  auto logger = builder.AddSystem<VectorLogSink>(1);

  const double time_period = 0.01;
  auto sys = builder.AddSystem(
      DiscreteTimeApproximation(Integrator<double>(1), time_period));
  // Connect a constant 1.0 to the discrete-time approximation of the
  // integrator
  builder.Connect(source->get_output_port(), sys->get_input_port());
  builder.Connect(sys->get_output_port(), logger->get_input_port());

  auto diagram = builder.Build();

  Simulator simulator(*diagram);
  const double tf = 1.001;
  simulator.AdvanceTo(tf);

  auto& log = logger->FindLog(simulator.get_context());
  for (int i = 0; i < log.num_samples(); ++i) {
    const double rem = std::remainder(log.sample_times()[i], time_period);
    const double area = (std::round(log.sample_times()[i] / time_period) +
                         (std::abs(rem) > 1e-12 ? 1 : 0)) *
                        time_period;
    ASSERT_NEAR(area, log.data().coeff(i), 1e-12);
  }
}

GTEST_TEST(DiscreteTimeApproximation, IntegrateRampTest) {
  DiagramBuilder<double> builder;
  auto source = builder.AddSystem<ConstantVectorSource>(
      Eigen::Matrix<double, 1, 1>::Ones());
  auto ramp = builder.AddSystem<Integrator>(1);
  builder.Connect(source->get_output_port(), ramp->get_input_port());

  const double time_period = 0.01;
  auto sys = builder.AddSystem(
      DiscreteTimeApproximation(Integrator<double>(1), time_period));
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

GTEST_TEST(DiscreteTimeApproximation, ResultSystemIsScalarConvertible) {
  const double time_period = 0.01;
  auto sys1 = DiscreteTimeApproximation(Integrator<double>(1), time_period);
  EXPECT_NO_THROW(sys1->ToAutoDiffXd());
  EXPECT_NO_THROW(sys1->Clone());
  auto sys2 = DiscreteTimeApproximation(Integrator<AutoDiffXd>(1), time_period);
  EXPECT_NO_THROW(sys2->ToScalarType<double>());
  EXPECT_NO_THROW(sys2->Clone());
}

class DiscreteTimeApproximatedMultibodyPlantTest : public ::testing::Test {
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

TEST_F(DiscreteTimeApproximatedMultibodyPlantTest, Construction) {
  auto continuous_context = continuous_sys_->CreateDefaultContext();
  auto discrete_context = discrete_sys_->CreateDefaultContext();

  EXPECT_TRUE(continuous_context->has_only_continuous_state());
  EXPECT_TRUE(discrete_context->has_only_discrete_state() &&
              discrete_context->num_discrete_state_groups() == 1);

  EXPECT_EQ(continuous_context->num_continuous_states(),
            discrete_context->get_discrete_state_vector().size());
}

TEST_F(DiscreteTimeApproximatedMultibodyPlantTest, InputPortsMatch) {
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

TEST_F(DiscreteTimeApproximatedMultibodyPlantTest, OutputPortsMatch) {
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

TEST_F(DiscreteTimeApproximatedMultibodyPlantTest, SynthesizeDiscreteTimeLQR) {
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

TEST_F(DiscreteTimeApproximatedMultibodyPlantTest, DiscretePlantSimulatable) {
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
