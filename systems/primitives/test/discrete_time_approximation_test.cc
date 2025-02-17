#include "drake/systems/primitives/discrete_time_approximation.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(DiscreteTimeApproximation, IntegrateConstantTest) {
  const double time_period = 0.01;
  auto sys = DiscreteTimeApproximation(Integrator<double>(1), time_period);
  auto context = sys->CreateDefaultContext();
  const double u = 1.0;
  sys->get_input_port().FixValue(context.get(), u);
  context->SetDiscreteState(Eigen::Matrix<double, 1, 1>::Zero());

  double value = 0.0;
  for (int i = 0; i < 100; ++i) {
    const DiscreteValues<double>& state =
        sys->EvalUniquePeriodicDiscreteUpdate(*context);
    context->SetDiscreteState(state);
    value += u * time_period;
    ASSERT_NEAR(value, sys->get_output_port().Eval(*context)[0], 1e-12);
  }
}

GTEST_TEST(DiscreteTimeApproximation, IntegrateRampTest) {
  using T = double;
  DiagramBuilder<T> builder;

  const System<T>* const source =
      builder.AddSystem<ConstantVectorSource>(Eigen::Matrix<T, 1, 1>::Ones());
  const System<T>* const ramp = builder.AddSystem<Integrator>(1);
  builder.Connect(source->get_output_port(), ramp->get_input_port());

  const double time_period = 0.01;
  const System<T>* const sys = builder.AddSystem(
      DiscreteTimeApproximation(Integrator<T>(1), time_period));
  // Connect a ramp of slope 1 to the discrete-time approximation of the
  // integrator. Since the discrete-time approximation has zero-order hold on
  // the input, the output is the area under a staircase.
  builder.Connect(ramp->get_output_port(), sys->get_input_port());
  builder.ExportOutput(sys->get_output_port());

  auto diagram = builder.Build();

  Simulator simulator(*diagram, diagram->CreateDefaultContext());
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
    DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    multibody::Parser parser(&plant, &scene_graph);
    std::string cartpole_urdf = R"(
    <?xml version="1.0"?>
    <robot xmlns="http://drake.mit.edu"
     xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
     name="CartPole">

      <link name="cart">
        <inertial>
          <origin xyz="0 0 .25" rpy="0 0 0" />
          <mass value="10" />
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />
        </inertial>
        <visual>
          <origin xyz="0 0 .25" rpy="0 0 0" />
          <geometry>
            <box size=".6 .3 .3" />
          </geometry>
          <material>
            <color rgba="0 1 0 1" />
          </material>
        </visual>
        <visual>
          <origin xyz=".15 0 .025" rpy="0 0 0" />
          <geometry>
            <sphere radius=".05" />
          </geometry>
          <material>
            <color rgba="0 0 0 1" />
          </material>
        </visual>
        <visual>
          <origin xyz="-.15 0 .025" rpy="0 0 0" />
          <geometry>
            <sphere radius=".05" />
          </geometry>
          <material>
            <color rgba="0 0 0 1" />
          </material>
        </visual>
      </link>

      <link name="pole">
        <inertial>
          <origin xyz="0 0 -.5" rpy="0 0 0" />
          <mass value="1" />
          <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />
        </inertial>
        <visual>
          <origin xyz="0 0 -.25" rpy="0 0 0" />
          <geometry>
             <cylinder length=".5" radius=".01" />
          </geometry>
          <material>
            <color rgba="1 0 0 1" />
          </material>
        </visual>
        <visual>
          <origin xyz="0 0 -.5" rpy="0 0 0" />
          <geometry>
             <sphere radius=".05" />
          </geometry>
          <material>
            <color rgba="0 0 1 1" />
          </material>
        </visual>
      </link>

      <joint name="x" type="prismatic">
        <parent link="world" />
        <child link="cart" />
        <origin xyz="0 0 0" />
        <axis xyz="1 0 0" />
      </joint>

      <joint name="theta" type="continuous">
        <parent link="cart" />
        <child link="pole" />
        <origin xyz="0 0 .25" />
        <axis xyz="0 -1 0" />
      </joint>

      <transmission type="SimpleTransmission" name="cart_force">
        <actuator name="force" />
        <joint name="x" />
        <mechanicalReduction>1</mechanicalReduction>
      </transmission>
    </robot>
    )";
    parser.AddModelsFromString(cartpole_urdf, "urdf");
    plant.Finalize();

    builder.ExportInput(plant.get_actuation_input_port(), "u");
    builder.ExportOutput(plant.get_state_output_port(), "x");
    builder.ExportOutput(scene_graph.get_query_output_port(), "query");

    continuous_sys_ = builder.Build();
    continuous_context_ = continuous_sys_->CreateDefaultContext();

    discrete_sys_ = DiscreteTimeApproximation(*continuous_sys_, time_period_);
    discrete_context_ = discrete_sys_->CreateDefaultContext();
  }

 protected:
  std::shared_ptr<System<double>> continuous_sys_;
  std::unique_ptr<Context<double>> continuous_context_;
  std::unique_ptr<System<double>> discrete_sys_;
  std::unique_ptr<Context<double>> discrete_context_;
  const double time_period_ = 0.01;
};

TEST_F(DiscreteTimeApproximatedMultibodyPlantTest, Construction) {
  EXPECT_TRUE(continuous_sys_->IsDifferentialEquationSystem());
  EXPECT_TRUE(discrete_sys_->IsDifferenceEquationSystem());

  EXPECT_EQ(continuous_context_->num_continuous_states(),
            discrete_context_->get_discrete_state_vector().size());
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

TEST_F(DiscreteTimeApproximatedMultibodyPlantTest, LQRControl) {
  const double theta_d = M_PI;
  Eigen::VectorXd xd(4);
  xd << 0, theta_d, 0, 0;
  Eigen::VectorXd ud(1);
  ud << 0;
  discrete_context_->SetDiscreteState(xd);
  discrete_sys_->get_input_port().FixValue(discrete_context_.get(), ud);

  Eigen::VectorXd Q_diag(4);
  Q_diag << 10, 10, 1, 1;
  Eigen::MatrixXd Q = Q_diag.asDiagonal();
  Eigen::VectorXd R_diag(1);
  R_diag << 1;
  Eigen::MatrixXd R = R_diag.asDiagonal();

  std::shared_ptr controller = controllers::LinearQuadraticRegulator(
      *discrete_sys_, *discrete_context_, Q, R);

  DiagramBuilder<double> builder;
  builder.AddSystem(continuous_sys_);
  builder.AddSystem(controller);
  auto zoh = builder.AddSystem<ZeroOrderHold>(time_period_, 1);
  builder.Connect(continuous_sys_->get_output_port(0),
                  controller->get_input_port());
  builder.Connect(controller->get_output_port(), zoh->get_input_port());
  builder.Connect(zoh->get_output_port(), continuous_sys_->get_input_port());
  builder.ExportOutput(continuous_sys_->GetOutputPort("query"), "query");
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();

  const double theta_init = M_PI * 0.9;
  Eigen::VectorXd x_init(4);
  x_init << 0, theta_init, 0, 0;
  diagram_context->SetContinuousState(x_init);

  Simulator simulator(*diagram, std::move(diagram_context));
  simulator.AdvanceTo(30);
  const double theta_final =
      simulator.get_context().get_continuous_state_vector()[1];
  // Check discrete-time controller stabilizes continuous-time cart-pole.
  EXPECT_LT(std::abs(theta_final - theta_d), std::abs(theta_init - theta_d));
}

}  // namespace
}  // namespace systems
}  // namespace drake
