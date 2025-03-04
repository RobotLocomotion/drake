#include "drake/systems/analysis/convex_integrator.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
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

using Eigen::VectorXd;
using Eigen::MatrixXd;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::JointActuator;
using multibody::JointActuatorIndex;
using systems::ConstantVectorSource;
using systems::FirstOrderTaylorApproximation;
using systems::controllers::PidController;
using visualization::AddDefaultVisualization;

class ConvexIntegratorTester {
 public:
  ConvexIntegratorTester() = delete;

  static void LinearizeExternalSystem(
      ConvexIntegrator<double>* integrator,
      LinearizedExternalSystem<double>* linear_sys) {
    integrator->LinearizeExternalSystem(linear_sys);
  }

  static LinearizedExternalSystem<double> GetLinearizedExternalSystem(
      ConvexIntegrator<double>* integrator) {
    return integrator->linearized_external_system_;
  }

  static void CalcImplicitExternalSystemData(
      ConvexIntegrator<double>* integrator,
      const LinearizedExternalSystem<double>& linear_sys, const double& h,
      ImplicitExternalSystemData<double>* implicit_data) {
    integrator->CalcImplicitExternalSystemData(linear_sys, h, implicit_data);
  }

  static ImplicitExternalSystemData<double> GetImplicitExternalSystemData(
      ConvexIntegrator<double>* integrator) {
    return integrator->implicit_external_system_data_;
  }
};

// MJCF model of a simple double pendulum
const char double_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="double_pendulum">
<worldbody>
  <body>
  <joint type="hinge" axis="0 1 0" pos="0 0 0.1" damping="1e-3"/>
  <geom type="capsule" size="0.01 0.1"/>
  <body>
    <joint type="hinge" axis="0 1 0" pos="0 0 -0.1" damping="1e-3"/>
    <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
  </body>
  </body>
</worldbody>
</mujoco> 
)""";

// MJCF model of a cylinder falling on a table
const char cylinder_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="robot">
<worldbody>
  <geom name="table_top" type="box" pos="0.0 0.0 0.0" size="0.55 1.1 0.05" rgba="0.9 0.8 0.7 1"/>
  <body>
    <joint type="free"/>
    <geom name="object" type="cylinder" pos="0.0 0.0 0.5" euler="80 0 0" size="0.1 0.1" rgba="1.0 1.0 1.0 1.0"/>
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
      <joint name="joint1" type="hinge" axis="0 1 0" pos="0 0 0.1" damping="1e-3"/>
      <geom type="capsule" size="0.01 0.1"/>
      <body>
        <joint name="joint2" type="hinge" axis="0 1 0" pos="0 0 -0.1" damping="1e-3"/>
        <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="joint1"/>
    <motor joint="joint2"/>
  </actuator>
</mujoco>
)""";

// Simulate a simple double pendulum system with the convex integrator. Play
// the sim back over meshcat so we can see what's going on.
GTEST_TEST(ConvexIntegratorTest, DoublePendulumSim) {
  // Start meshcat
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  // Set up the system diagram
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(double_pendulum_xml, "xml");
  plant.Finalize();
  AddDefaultVisualization(&builder, meshcat);
  auto diagram = builder.Build();

  // Create context and set the initial state
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  Eigen::Vector2d q0(3.0, 0.1);
  plant.SetPositions(&plant_context, q0);

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
  simulator.Initialize();

  // Simulate for a few seconds
  const int fps = 64;
  meshcat->StartRecording(fps);
  simulator.AdvanceTo(0.1);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;
}

// Run a short simulation with the convex integrator. Most useful as a quick
// sanity check.
GTEST_TEST(ConvexIntegratorTest, ShortSim) {
  // Time step
  const double h = 0.01;

  // Create a continuous-time system
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(cylinder_xml, "xml");
  plant.Finalize();
  auto diagram = builder.Build();

  Simulator<double> simulator(*diagram);
  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.set_maximum_step_size(h);
  simulator.Initialize();

  simulator.AdvanceTo(1.0);
}

// Simulate a cylinder falling on a table with the convex integrator. Play
// the sim back over meshcat so we can see what's going on.
GTEST_TEST(ConvexIntegratorTest, CylinderSim) {
  // Start meshcat
  auto meshcat = std::make_shared<drake::geometry::Meshcat>();

  // Set up the system diagram
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph).AddModelsFromString(cylinder_xml, "xml");
  plant.Finalize();
  AddDefaultVisualization(&builder, meshcat);

  // Set up hydroelastic contact
  geometry::SceneGraphConfig scene_graph_config;
  scene_graph_config.default_proximity_properties.compliance_type = "compliant";
  scene_graph.set_config(scene_graph_config);
  auto diagram = builder.Build();

  // Set up the simulator
  Simulator<double> simulator(*diagram);
  SimulatorConfig config;
  config.target_realtime_rate = 1.0;
  config.publish_every_time_step = true;
  ApplySimulatorConfig(config, &simulator);

  // Set the integrator
  ConvexIntegrator<double>& integrator =
      simulator.reset_integrator<ConvexIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  simulator.Initialize();

  // Simulate for a few seconds
  const int fps = 32;
  meshcat->StartRecording(fps);
  simulator.AdvanceTo(10.0);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;
}

// Run tests with an actuated pendulum and an external (PID) controller
GTEST_TEST(ConvexIntegratorTest, ActuatedPendulum) {
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

  VectorXd Kp(2), Kd(2), Ki(2);
  Kp << 0.2, 0.2;
  Kd << 0.1, 0.1;
  Ki << 0.01, 0.01;
  auto ctrl = builder.AddSystem<PidController>(Kp, Kd, Ki);

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
  integrator.set_maximum_step_size(0.1);
  simulator.Initialize();

  // Linearize the non-plant system dynamics around the current state
  LinearizedExternalSystem<double> linear_sys =
      ConvexIntegratorTester::GetLinearizedExternalSystem(&integrator);
  ConvexIntegratorTester::LinearizeExternalSystem(&integrator, &linear_sys);

  // Reference linearization
  const Context<double>& ctrl_context =
      ctrl->GetMyContextFromRoot(simulator.get_context());
  auto true_linearization = FirstOrderTaylorApproximation(
      *ctrl, ctrl_context, ctrl->get_input_port_estimated_state().get_index(),
      ctrl->get_output_port().get_index());

  // Confirm that our finite difference linearization is close to the reference
  const double kTolerance = std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(CompareMatrices(linear_sys.A, true_linearization->A(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(linear_sys.B, true_linearization->B(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(linear_sys.C, true_linearization->C(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(linear_sys.D, true_linearization->D(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(linear_sys.f0, true_linearization->f0(),
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(linear_sys.g0, true_linearization->y0(),
                              kTolerance, MatrixCompareType::relative));

  // Compute implicit integration data for the external system
  const double h = 0.01;
  ImplicitExternalSystemData<double> implicit_data = 
      ConvexIntegratorTester::GetImplicitExternalSystemData(&integrator);
  ConvexIntegratorTester::CalcImplicitExternalSystemData(&integrator,
      linear_sys, h, &implicit_data);    

  // Reference implicit integration data
  const MatrixXd A = true_linearization->A();
  const MatrixXd B = true_linearization->B();
  const MatrixXd C = true_linearization->C();
  const MatrixXd D = true_linearization->D();
  const VectorXd f0 = true_linearization->f0();
  const VectorXd g0 = true_linearization->y0();
  const VectorXd z = ctrl_context.get_continuous_state()
                        .get_misc_continuous_state()
                        .CopyToVector();
  const VectorXd q =
      plant.GetPositions(plant.GetMyContextFromRoot(simulator.get_context()));
  const MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());

  const MatrixXd H = h * (I - h * A).inverse() * B;
  const VectorXd h0 = (I - h * A).inverse() * (z + h * f0);
  const MatrixXd P = D + h * C * (I - h * A).inverse() * B;
  const VectorXd p = C * (I - h * A).inverse() * (z + h * f0) + g0;
  const MatrixXd Pq = P.leftCols(plant.num_positions());
  const MatrixXd Pv = P.rightCols(plant.num_velocities());
  const MatrixXd K = Pv;
  const VectorXd k0 = p + Pq * q;

  // Check that we're close to the reference
  EXPECT_TRUE(CompareMatrices(implicit_data.H, H, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(implicit_data.h0, h0, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(implicit_data.K, K, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(implicit_data.k0, k0, kTolerance,
                              MatrixCompareType::relative));

  // Set up another diagram that uses SAP and an implicit PD controller
  DiagramBuilder<double> builder2;
  auto [plant2, scene_graph2] = AddMultibodyPlantSceneGraph(&builder2, h);
  Parser(&plant2, &scene_graph2)
      .AddModelsFromString(actuated_pendulum_xml, "xml");

  for (JointActuatorIndex actuator_index : plant2.GetJointActuatorIndices()) {
    JointActuator<double>& actuator =
        plant2.get_mutable_joint_actuator(actuator_index);
    actuator.set_controller_gains({Kp(0), Kd(0)});
  }
  plant2.Finalize();
  auto target_state2 = builder2.AddSystem<ConstantVectorSource<double>>(x_nom);
  builder2.Connect(
    target_state2->get_output_port(),
    plant2.get_desired_state_input_port(plant2.GetModelInstanceByName("robot"))
  );
  auto diagram2 = builder2.Build();
  (void)diagram2;

  // Set an interesting initial state for both systems
  VectorXd q0(2), v0(2);
  q0 << 0.1, 0.2;
  v0 << 0.3, 0.4;

  auto diagram2_context = diagram2->CreateDefaultContext();
  Context<double>& plant2_context =
      plant2.GetMyMutableContextFromRoot(diagram2_context.get());
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

  plant.SetPositions(&plant_context, q0);
  plant.SetVelocities(&plant_context, v0);
  plant2.SetPositions(&plant2_context, q0);
  plant2.SetVelocities(&plant2_context, v0);

  // Get the SAP Hessian for each problem

  // Simulate for a few seconds
  const int fps = 32;
  meshcat->StartRecording(fps);
  simulator.AdvanceTo(10.0);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  std::cout << std::endl;
  PrintSimulatorStatistics(simulator);
  std::cout << std::endl;
}

}  // namespace systems
}  // namespace drake
