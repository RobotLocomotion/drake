#include "drake/systems/analysis/convex_integrator.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
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
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace systems {
namespace analysis_test {

using Eigen::VectorXd;
using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::MultibodyPlant;
using multibody::Parser;
using systems::ConstantVectorSource;
using systems::controllers::PidController;
using visualization::AddDefaultVisualization;

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

// MJCF model of an actuated pendulum
const char actuated_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <body>
      <joint name="joint" type="hinge" axis="0 1 0" pos="0 0 0.1" damping="1e-3"/>
      <geom type="capsule" size="0.01 0.1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="joint"/>
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

  VectorXd x_nom(2);
  x_nom << M_PI_2, 0.0;
  auto target_state = builder.AddSystem<ConstantVectorSource<double>>(x_nom);

  VectorXd Kp(1), Kd(1), Ki(1);
  Kp << 0.2;
  Kd << 0.1;
  Ki << 0.01;
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
  config.integration_scheme = "convex";
  config.accuracy = 0.1;
  ApplySimulatorConfig(config, &simulator);
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

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
