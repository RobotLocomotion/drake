#include "drake/systems/analysis/convex_integrator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Joint;
using multibody::JointActuator;
using multibody::JointActuatorIndex;
using multibody::JointIndex;
using multibody::Parser;
using multibody::SpatialInertia;
using systems::ConstantVectorSource;
using systems::FirstOrderTaylorApproximation;
using systems::controllers::PidController;
using visualization::AddDefaultVisualization;

// MJCF model of a simple double pendulum
const char double_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="double_pendulum">
<worldbody>
  <body>
  <joint type="hinge" axis="0 1 0" pos="0 0 0.1" damping="0.001"/>
  <geom type="capsule" size="0.01 0.1"/>
  <body>
    <joint type="hinge" axis="0 1 0" pos="0 0 -0.1" damping="0.001"/>
    <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
  </body>
  </body>
</worldbody>
</mujoco> 
)""";

// MJCF model of an actuated double pendulum
const char actuated_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <body>
      <joint name="joint1" type="hinge" axis="0 1 0" pos="0 0 0.1"/>
      <geom type="capsule" size="0.01 0.1"/>
      <body>
        <joint name="joint2" type="hinge" axis="0 1 0" pos="0 0 -0.1"/>
        <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="joint1" ctrlrange="-2 2"/>
    <motor joint="joint2" ctrlrange="-3 3"/>
  </actuator>
</mujoco>
)""";

// MJCF of a simple pendulum with joint limits
const char limited_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <body>
      <joint name="joint1" type="hinge" axis="0 1 0" pos="0 0 0.1" range="10 90"/>
      <geom type="capsule" size="0.01 0.1"/>
    </body>
  </worldbody>
</mujoco>
)""";

// MJCF of a double pendulum with actuation only on joint2.
const char joint2_actuation_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <body>
      <joint name="joint1" type="hinge" axis="0 1 0" pos="0 0 0.1"/>
      <geom type="capsule" size="0.01 0.1"/>
      <body>
        <joint name="joint2" type="hinge" axis="0 1 0" pos="0 0 -0.1"/>
        <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="joint2" ctrlrange="-3 3"/>
  </actuator>
</mujoco>
)""";

// MJCF of a double pendulum with a quaternion floating base. Used to test 
// non-SPD Hessian issues.
const char floating_double_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <geom name="floor" type="plane" pos="0 0 0" size="5 5 0.1" rgba="0.8 0.9 0.8 1"/>
    <body name="link1" pos="0 0 1.0">
      <joint name="joint1" type="free"/>
      <geom name="geom1" type="capsule" size="0.1 0.5" pos="0 0 0" rgba="0.9 0.1 0.1 1"/>
      <body name="link2" pos="0 0 0.5">
        <joint name="joint2" type="hinge" axis="1 0 0" range="-90 90" damping="1"/>
        <geom name="geom2" type="capsule" size="0.1 0.5" pos="0 0.0 0.5" rgba="0.1 0.1 0.9 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
)""";

class ConvexIntegratorTester {
 public:
  ConvexIntegratorTester() = delete;

  static void LinearizeExternalSystem(ConvexIntegrator<double>* integrator,
                                      const double h, VectorXd* Ku,
                                      VectorXd* bu, VectorXd* Ke,
                                      VectorXd* be) {
    integrator->LinearizeExternalSystem(h, Ku, bu, Ke, be);
  }
};

GTEST_TEST(ConvexIntegratorTest, TestConstruction) {
  // Create a simple system
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(double_pendulum_xml, "xml");
  plant.Finalize();
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Set up the integrator, without specifying the plant.
  ConvexIntegrator<double> integrator(*diagram, context.get());
  integrator.set_maximum_step_size(0.01);

  // Integrator initialization should fail unless the plant is set correctly.
  EXPECT_THROW(integrator.Initialize(), std::runtime_error);

  MultibodyPlant<double> wrong_plant(0.0);
  wrong_plant.Finalize();
  integrator.set_plant(&wrong_plant);
  EXPECT_THROW(integrator.Initialize(), std::runtime_error);

  integrator.set_plant(&plant);
  integrator.Initialize();
}

GTEST_TEST(ConvexIntegratorTest, TestStep) {
  // TODO(vincekurtz): update this test to include joint damping
  // Create a simple system
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(double_pendulum_xml, "xml");
  plant.Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  // Set up the integrator
  ConvexIntegrator<double> integrator(*diagram, diagram_context.get());
  integrator.set_plant(&plant);
  integrator.set_maximum_step_size(0.01);
  integrator.set_fixed_step_mode(true);
  integrator.Initialize();

  // Set initial conditions
  VectorXd q0(2);
  VectorXd v0(2);
  q0 << 0.1, 0.2;
  v0 << 0.3, 0.4;
  plant.SetPositions(&plant_context, q0);
  plant.SetVelocities(&plant_context, v0);

  // Perform a step
  const double dt = 0.01;
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(dt));
  EXPECT_NEAR(plant_context.get_time(), dt,
              std::numeric_limits<double>::epsilon());

  // Discrete-time reference
  MultibodyPlant<double> reference_plant(dt);
  Parser(&reference_plant).AddModelsFromString(double_pendulum_xml, "xml");
  reference_plant.Finalize();
  auto reference_context = reference_plant.CreateDefaultContext();
  reference_plant.SetPositions(reference_context.get(), q0);
  reference_plant.SetVelocities(reference_context.get(), v0);

  Simulator<double> simulator(reference_plant, std::move(reference_context));
  simulator.Initialize();
  simulator.AdvanceTo(dt);
  EXPECT_NEAR(simulator.get_context().get_time(), dt,
              std::numeric_limits<double>::epsilon());

  const VectorXd& q_ref = reference_plant.GetPositions(simulator.get_context());
  const VectorXd& q = plant.GetPositions(plant_context);
  const double kTol = std::sqrt(std::numeric_limits<double>::epsilon());

  EXPECT_TRUE(CompareMatrices(q, q_ref, kTol, MatrixCompareType::relative));
}

/* A single free body spins away with a massive angular velocity. The quaternion
portion of the state should remain normalized. */
GTEST_TEST(ConvexIntegratorTest, TestQuaternions) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  // Add a free body with a quaternion state.
  SpatialInertia<double> M_BBcm =
      SpatialInertia<double>::SolidSphereWithMass(0.1, 0.1);
  plant.AddRigidBody("Ball", M_BBcm);
  plant.Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  // Set initial velocity to be spinning fast
  VectorXd v0(6);
  v0 << 1e3, 2e3, 3e3, 0, 0, 0;
  plant.SetVelocities(&plant_context, v0);
  fmt::print("Initial velocity: {}\n", fmt_eigen(v0.transpose()));

  VectorXd q0(7);
  q0 << 0.707, 0.707, 0, 0, 0, 0, 0;
  plant.SetPositions(&plant_context, q0);

  // Set up the integrator
  Simulator<double> simulator(*diagram, std::move(diagram_context));
  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.set_plant(&plant);
  integrator.set_maximum_step_size(0.1);  // fairly large dt
  integrator.set_fixed_step_mode(true);

  // Simulate for a few seconds
  simulator.Initialize();
  simulator.AdvanceTo(5.0);

  VectorXd v = plant.GetVelocities(plant_context);
  VectorXd q = plant.GetPositions(plant_context);
  fmt::print("Final velocity: {}\n", fmt_eigen(v.transpose()));
  fmt::print("Final position: {}\n", fmt_eigen(q.transpose()));

  VectorXd quat = q.head(4);
  fmt::print("Quat norm: {}\n", quat.norm());
  EXPECT_NEAR(quat.norm(), 1.0, 1e-6);
}

// Run tests with an actuated pendulum and an external (PID) controller
GTEST_TEST(ConvexIntegratorTest, ActuatedPendulum) {
  // Some options
  const double h = 0.01;
  VectorXd Kp(2), Kd(2), Ki(2);
  Kp << 0.24, 0.19;
  Kd << 0.35, 0.3;
  Ki << 0.0, 0.0;

  // Start meshcat
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  // Set up the system diagram
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph)
      .AddModelsFromString(actuated_pendulum_xml, "xml");
  plant.Finalize();

  AddDefaultVisualization(&builder, meshcat);

  VectorXd x_nom(4);
  x_nom << M_PI_2, M_PI_2, 0.0, 0.0;
  auto target_state = builder.AddSystem<ConstantVectorSource<double>>(x_nom);

  // PD controller is an external system
  auto ctrl = builder.AddSystem<PidController>(Kp, Ki, Kd);

  builder.Connect(target_state->get_output_port(),
                  ctrl->get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(),
                  ctrl->get_input_port_estimated_state());
  builder.Connect(ctrl->get_output_port(), plant.get_actuation_input_port());

  auto diagram = builder.Build();

  // Set up the simulator
  Simulator<double> simulator(*diagram);
  SimulatorConfig config;
  config.target_realtime_rate = 1.0;
  config.publish_every_time_step = true;
  ApplySimulatorConfig(config, &simulator);

  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.get_mutable_solver_parameters().enable_hessian_reuse = false;
  integrator.get_mutable_solver_parameters().print_solver_stats = true;
  integrator.set_plant(&plant);
  integrator.set_fixed_step_mode(true);
  integrator.set_maximum_step_size(h);

  // Set an interesting initial state
  VectorXd q0(2), v0(2);
  q0 << 0.1, 0.2;
  v0 << 0.3, 0.4;
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());
  plant.SetPositions(&plant_context, q0);
  plant.SetVelocities(&plant_context, v0);
  simulator.Initialize();

  // Linearize the non-plant system dynamics around the current state
  const int nv = plant.num_velocities();
  VectorXd K(nv), b(nv);
  VectorXd Ke(nv), be(nv);  // unused, for linearizing other input ports
  ConvexIntegratorTester::LinearizeExternalSystem(&integrator, h, &K, &b, &Ke,
                                                  &be);

  // Reference linearization via autodiff
  const Context<double>& ctrl_context =
      ctrl->GetMyContextFromRoot(simulator.get_context());
  auto true_linearization = FirstOrderTaylorApproximation(
      *ctrl, ctrl_context, ctrl->get_input_port_estimated_state().get_index(),
      ctrl->get_output_port().get_index());

  const MatrixXd B = plant.MakeActuationMatrix();
  const MatrixXd& D = true_linearization->D();
  const MatrixXd Du = D.rightCols(2) + h * D.leftCols(2);  // N(q) = I
  const VectorXd u0 = plant.get_actuation_input_port().Eval(plant_context) -
                      Du * plant.GetVelocities(plant_context);

  const VectorXd K_ref = -(B * Du).diagonal();
  const VectorXd b_ref = B * u0;

  // Confirm that our finite difference linearization is close to the reference
  const double kTolerance = std::sqrt(std::numeric_limits<double>::epsilon());

  EXPECT_TRUE(
      CompareMatrices(K, K_ref, kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(b, b_ref, kTolerance, MatrixCompareType::relative));

  // Compute the cost, gradient and Hessian analytically, and check that
  // these match what we get from the PooledSap model.
  //
  // Cost: ℓ = 1/2 v'Mv − r v + h/2 ∑(kᵢvᵢ−bᵢ)²/kᵢ =
  //         = 1/2 v'Mv − r v + h (1/2 v'Kv - b v) + h/2 b'K⁻¹b
  // Gradient: dℓ/dv = Mv − r + h (Kv - b)
  // Hessian: d²ℓ/dv² = M + h K
  const VectorXd v = v0;
  MatrixXd M(nv, nv);
  plant.CalcMassMatrix(plant_context, &M);
  MultibodyForces<double> f_ext(plant);
  plant.CalcForceElementsContribution(plant_context, &f_ext);
  const VectorXd r =
      -h * plant.CalcInverseDynamics(plant_context, -v0 / h, f_ext);

  const VectorXd K_inv = K_ref.cwiseInverse();
  const double l_ref =
      0.5 * v.dot(M * v) - r.dot(v) + h * 0.5 * v.dot(K_ref.asDiagonal() * v) -
      h * b_ref.dot(v) + 0.5 * h * b_ref.dot(K_inv.asDiagonal() * b_ref);
  const VectorXd dl_ref = M * v - r + h * K_ref.asDiagonal() * v - h * b_ref;
  const MatrixXd H_ref =
      M + h * K_ref.asDiagonal() * MatrixXd::Identity(nv, nv);

  const PooledSapBuilder<double>& sap_builder = integrator.builder();
  PooledSapModel<double>& model = integrator.get_model();
  SapData<double>& data = integrator.get_data();
  sap_builder.UpdateModel(plant_context, h, &model);
  sap_builder.AddActuationGains(K, b, &model);
  model.ResizeData(&data);
  model.CalcData(v, &data);
  const VectorXd dl = data.cache().gradient;
  const double l = data.cache().cost;
  const MatrixXd H = model.MakeHessian(data)->MakeDenseMatrix();

  EXPECT_TRUE(
      CompareMatrices(dl, dl_ref, kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(
      CompareMatrices(H, H_ref, kTolerance, MatrixCompareType::relative));
  EXPECT_NEAR(l, l_ref, kTolerance);

  // Simulate for a few seconds to make sure nothing breaks
  const int fps = 32;
  meshcat->StartRecording(fps);
  simulator.set_target_realtime_rate(0.0);
  simulator.AdvanceTo(2.0);
  meshcat->StopRecording();
  meshcat->PublishRecording();
  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;
}

// Test implicit joint effort limits
GTEST_TEST(ConvexIntegratorTest, EffortLimits) {
  // Set up the a system model with effort limits
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph)
      .AddModelsFromString(actuated_pendulum_xml, "xml");
  plant.Finalize();

  // Connect a high-gain PD controller
  VectorXd Kp(2), Kd(2), Ki(2);
  Kp << 1e3, 1e3;
  Kd << 0.1, 0.1;
  Ki << 0.0, 0.0;
  auto ctrl = builder.AddSystem<PidController>(Kp, Ki, Kd);

  VectorXd x_nom(4);
  x_nom << M_PI_2, M_PI_2, 0.0, 0.0;
  auto target_state = builder.AddSystem<ConstantVectorSource<double>>(x_nom);

  builder.Connect(target_state->get_output_port(),
                  ctrl->get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(),
                  ctrl->get_input_port_estimated_state());
  builder.Connect(ctrl->get_output_port(), plant.get_actuation_input_port());

  // Compile the system diagram
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Get the actuator effort limits
  VectorXd effort_limits(2);
  for (JointActuatorIndex a : plant.GetJointActuatorIndices()) {
    const JointActuator<double>& actuator = plant.get_joint_actuator(a);
    const int i = actuator.input_start();
    const int n = actuator.num_inputs();
    effort_limits.segment(i, n) =
        VectorXd::Constant(n, actuator.effort_limit());
  }

  // Set up the integrator
  ConvexIntegrator<double> integrator(*diagram, diagram_context.get());
  integrator.set_plant(&plant);
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(true);
  integrator.Initialize();

  const VectorXd q0 = plant.GetPositions(plant_context);
  const VectorXd v0 = plant.GetVelocities(plant_context);

  // Compute some dynamics terms
  MatrixXd M(2, 2);
  VectorXd k(2);
  MultibodyForces<double> f_ext(plant);
  plant.CalcMassMatrix(plant_context, &M);
  plant.CalcForceElementsContribution(plant_context, &f_ext);
  k = plant.CalcInverseDynamics(plant_context, VectorXd::Zero(2), f_ext);

  // Simulate for a step
  const double h = 0.01;
  EXPECT_TRUE(integrator.IntegrateWithSingleFixedStepToTime(h));

  // Compare requested and applied actuator forces
  const VectorXd u_req = plant.get_actuation_input_port().Eval(plant_context);
  EXPECT_TRUE(u_req[0] > effort_limits[0]);
  EXPECT_TRUE(u_req[1] > effort_limits[1]);

  // TODO(vincekurtz): report u_app in plant.get_net_actuation_output_port()
  const VectorXd v = plant.GetVelocities(plant_context);
  const VectorXd u_app = M * (v - v0) / h + k;

  const double kTol = std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(u_app[0] <= effort_limits[0] + kTol);
  EXPECT_TRUE(u_app[1] <= effort_limits[1] + kTol);
}

// Check that we can enforce joint limits
GTEST_TEST(ConvexIntegratorTest, JointLimits) {
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  // Set up the a system model with joint limits
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(limited_pendulum_xml, "xml");
  plant.Finalize();

  AddDefaultVisualization(&builder, meshcat);

  // Compile the system diagram
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  plant.SetPositions(&plant_context, Vector1d(M_PI_2));

  // Set up the simulator
  Simulator<double> simulator(*diagram, std::move(diagram_context));
  SimulatorConfig config;
  config.target_realtime_rate = 1.0;
  config.publish_every_time_step = true;
  ApplySimulatorConfig(config, &simulator);

  // Set the integrator
  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(true);
  integrator.set_plant(&plant);
  simulator.Initialize();

  // Simulate for a few seconds
  const int fps = 64;
  meshcat->StartRecording(fps);
  simulator.AdvanceTo(0.5);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  // Check that the final joint position is right at the limit
  const VectorXd q = plant.GetPositions(plant_context);
  const double q_min =
      plant.GetJointByName("joint1").position_lower_limits()[0];
  EXPECT_NEAR(q[0], q_min, 1e-3);

  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;
}

GTEST_TEST(ConvexIntegratorTest, PendulumWithCoupler) {
  // Some options
  const double h = 0.01;
  VectorXd Kp(1), Kd(1), Ki(1);
  Kp << 1000.0;
  Kd << 30.0;
  Ki << 0.0;
  // Projection matrix to control only states on Joint 2.
  MatrixXd Px(2, 4);
  // clang-format off
  Px << 0, 1, 0, 0,
        0, 0, 0, 1;
  // clang-format on

  // Start meshcat
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  // Set up the system diagram
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph)
      .AddModelsFromString(joint2_actuation_pendulum_xml, "xml");
  plant.AddCouplerConstraint(plant.GetJointByName("joint1"),
                             plant.GetJointByName("joint2"), -2);
  plant.Finalize();

  AddDefaultVisualization(&builder, meshcat);

  VectorXd x_nom(2);
  x_nom << M_PI_2, 0.0;
  auto target_state = builder.AddSystem<ConstantVectorSource<double>>(x_nom);

  // PD controller is an external system
  auto ctrl = builder.AddSystem<PidController>(Px, Kp, Ki, Kd);

  builder.Connect(target_state->get_output_port(),
                  ctrl->get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(),
                  ctrl->get_input_port_estimated_state());
  builder.Connect(ctrl->get_output_port(), plant.get_actuation_input_port());

  auto diagram = builder.Build();

  // Set up the simulator
  Simulator<double> simulator(*diagram);
  SimulatorConfig config;
  config.target_realtime_rate = 1.0;
  config.publish_every_time_step = true;
  ApplySimulatorConfig(config, &simulator);

  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.get_mutable_solver_parameters().enable_hessian_reuse = false;
  integrator.get_mutable_solver_parameters().print_solver_stats = true;
  integrator.set_plant(&plant);
  integrator.set_fixed_step_mode(true);
  integrator.set_maximum_step_size(h);

  // Set an interesting initial state
  VectorXd q0(2), v0(2);
  q0 << 0.1, 0.2;
  v0 << 0.3, 0.4;
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());
  plant.SetPositions(&plant_context, q0);
  plant.SetVelocities(&plant_context, v0);
  simulator.Initialize();

  // Simulate for a few seconds to make sure nothing breaks
  const int fps = 32;
  meshcat->StartRecording(fps);
  simulator.set_target_realtime_rate(0.5);
  simulator.AdvanceTo(2.0);
  meshcat->StopRecording();
  meshcat->PublishRecording();
  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;

  // Sanity check steady state verifies coupler constraint.
  const VectorXd q = plant.GetPositions(plant_context);
  const double g = q[0] + 2.0 * q[1];
  EXPECT_NEAR(g, 0.0, 4e-6);
}

// Smoke test for letting a franka arm just flop down freely under gravity.
GTEST_TEST(ConvexIntegratorTest, FrankaFreeFall) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  const std::string panda_url =
      "package://drake_models/franka_description/urdf/panda_arm.urdf";
  Parser(&plant).AddModelsFromUrl(panda_url);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));
  plant.Finalize();
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  SimulatorConfig config;
  config.target_realtime_rate = 1.0;
  config.publish_every_time_step = true;
  ApplySimulatorConfig(config, &simulator);

  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.get_mutable_solver_parameters().enable_hessian_reuse = true;
  integrator.get_mutable_solver_parameters().print_solver_stats = true;
  integrator.set_plant(&plant);
  integrator.set_fixed_step_mode(true);
  integrator.set_maximum_step_size(0.01);

  VectorXd q0 = VectorXd::Zero(plant.num_positions());
  q0(1) = -M_PI_4;
  q0(3) = -M_PI_2;
  q0(5) = M_PI_4;
  q0(6) = -M_PI_4;
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());
  plant.SetPositions(&plant_context, q0);
  simulator.Initialize();
  simulator.set_target_realtime_rate(0.0);

  simulator.AdvanceTo(1.0);
  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;
}

// Smoke test for a floating double pendulum, to test non-SPD Hessian issues.
GTEST_TEST(ConvexIntegratorTest, FloatingDoublePendulum) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant).AddModelsFromString(floating_double_pendulum_xml, "xml");
  plant.Finalize();
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  SimulatorConfig config;
  config.target_realtime_rate = 0.0;
  config.publish_every_time_step = true;
  ApplySimulatorConfig(config, &simulator);

  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.get_mutable_solver_parameters().print_solver_stats = true;
  integrator.set_plant(&plant);
  integrator.set_fixed_step_mode(true);
  integrator.set_maximum_step_size(0.01);

  simulator.Initialize();
  simulator.AdvanceTo(1.0);
  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;
}

}  // namespace systems
}  // namespace drake
