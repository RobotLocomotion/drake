#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/scene_graph_config.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/joint_stiffness_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace examples {
namespace {

using drake::geometry::Meshcat;
using drake::geometry::SceneGraphConfig;
using drake::multibody::CenicIntegrator;
using drake::multibody::JointActuator;
using drake::multibody::JointActuatorIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::PackageMap;
using drake::multibody::Parser;
using drake::multibody::contact_solvers::icf::IcfSolverParameters;
using drake::multibody::contact_solvers::internal::SapHessianFactorizationType;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::internal::CompliantContactManager;
using drake::systems::ConstantVectorSource;
using drake::systems::DiagramBuilder;
using drake::systems::IntegratorBase;
using drake::systems::PrintSimulatorStatistics;
using drake::systems::Simulator;
using drake::systems::SimulatorConfig;
using drake::systems::controllers::InverseDynamicsController;
using drake::systems::controllers::JointStiffnessController;
using drake::systems::controllers::PidController;
using drake::visualization::ApplyVisualizationConfig;
using drake::visualization::VisualizationConfig;
using drake::yaml::LoadYamlFile;
using drake::yaml::LoadYamlOptions;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* Controller selection and config. */
struct ControllerConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(type));
    a->Visit(DRAKE_NVP(kp));
    a->Visit(DRAKE_NVP(ki));
    a->Visit(DRAKE_NVP(kd));
  }

  std::string type{"pid"};
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
};

/* All configuration settings for this example. */
struct Config {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(simulation_time));
    a->Visit(DRAKE_NVP(simulator_config));
    a->Visit(DRAKE_NVP(plant_config));
    a->Visit(DRAKE_NVP(scene_graph_config));
    a->Visit(DRAKE_NVP(visualization_config));
    a->Visit(DRAKE_NVP(icf_solver_config));
    a->Visit(DRAKE_NVP(controller_config));
    a->Visit(DRAKE_NVP(manipuland));
  }

  double simulation_time{0.0};
  SimulatorConfig simulator_config;
  MultibodyPlantConfig plant_config;
  SceneGraphConfig scene_graph_config;
  std::optional<VisualizationConfig> visualization_config;
  IcfSolverParameters icf_solver_config;
  ControllerConfig controller_config;
  bool manipuland{false};
};

int do_main() {
  const Config config = LoadYamlFile<Config>(
      PackageMap().ResolveUrl(
          "package://drake/examples/multibody/franka/config.yaml"),
      "config", std::nullopt,
      LoadYamlOptions{
          .allow_yaml_with_no_cpp = false,
          .allow_cpp_with_no_yaml = true,
          .retain_map_defaults = true,
      });

  // Build a generic multibody plant.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlant(
      config.plant_config, config.scene_graph_config, &builder);

  // Add the franka arm model.
  Parser parser(&plant);
  const std::string panda_url =
      "package://drake_models/franka_description/urdf/panda_arm_hand.urdf";
  const ModelInstanceIndex panda = parser.AddModelsFromUrl(panda_url)[0];
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));
  const ModelInstanceIndex robot = plant.GetModelInstanceByName("panda");

  // Add an object to hold, if requested.
  if (config.manipuland) {
    parser.AddModelsFromUrl("package://drake_models/ycb/003_cracker_box.sdf");
  }

  plant.Finalize();

  // Set up the meshcat visualizer.
  auto meshcat = std::make_shared<Meshcat>();
  if (config.visualization_config.has_value()) {
    ApplyVisualizationConfig(*config.visualization_config, &builder, nullptr,
                             &plant, &scene_graph, meshcat);
  }

  // Define a target state.
  VectorXd q_nom(plant.num_positions());
  VectorXd x_nom(plant.num_multibody_states());

  if (config.manipuland) {
    q_nom << 0, -0.4, 0.5, -M_PI_2, 0, M_PI_2, M_PI_4, 0.01, 0.01,  // robot
        0.707, 0.707, 0, 0, 0.55, 0.0, 0.43;                        // box
  } else {
    q_nom << 0, -0.4, 0.5, -M_PI_2, 0, M_PI_2, 0, 0.01, 0.01;
  }
  x_nom << q_nom, VectorXd::Zero(plant.num_velocities());

  // Set controller gains.
  MatrixXd Px = MatrixXd::Identity(plant.num_multibody_states(),
                                   plant.num_multibody_states());

  if (config.manipuland) {
    Px.resize(18, 31);
    Px.setZero();
    Px.block<9, 9>(0, 0) = MatrixXd::Identity(9, 9);
    Px.block<9, 9>(9, 16) = MatrixXd::Identity(9, 9);
  }

  // System than outputs desired joint positions and velocities.
  auto nominal_state_source =
      builder.AddSystem<ConstantVectorSource>(Px * x_nom);

  // Set up a simplified plant model for the controller, which does not include
  // the manipuland.
  MultibodyPlant<double> control_plant(1.0);
  Parser(&control_plant).AddModelsFromUrl(panda_url);
  control_plant.WeldFrames(control_plant.world_frame(),
                           control_plant.GetFrameByName("panda_link0"));
  control_plant.Finalize();

  // Create and connect the controller.
  if (plant.time_step() > 0.0) {
    // We'll use a discrete-time implicit PID controller.
    if (config.controller_config.type != "pid") {
      throw std::logic_error(
          "Discrete-time simulation only supports an implicit PID controller.");
    }

    // Set up implicit PD controller gains.
    DRAKE_THROW_UNLESS(config.controller_config.ki == 0.0);
    for (JointActuatorIndex actuator_index : plant.GetJointActuatorIndices()) {
      JointActuator<double>& actuator =
          plant.get_mutable_joint_actuator(actuator_index);
      actuator.set_controller_gains(
          {config.controller_config.kp, config.controller_config.kd});
    }

    builder.Connect(nominal_state_source->get_output_port(),
                    plant.get_desired_state_input_port(robot));

    // Force SAP to use dense algebra.
    SapSolverParameters sap_parameters;
    sap_parameters.linear_solver_type = SapHessianFactorizationType::kDense;

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    CompliantContactManager<double>* contact_manager =
        owned_contact_manager.get();
    plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    contact_manager->set_sap_solver_parameters(sap_parameters);
  } else {
    VectorXd Kp(9), Kd(9), Ki(9);
    Kp.fill(config.controller_config.kp);
    Kd.fill(config.controller_config.kd);
    Ki.fill(config.controller_config.ki);
    if (config.controller_config.type == "pid") {
      auto controller = builder.AddSystem<PidController>(Px, Kp, Ki, Kd);
      builder.Connect(nominal_state_source->get_output_port(),
                      controller->get_input_port_desired_state());
      builder.Connect(plant.get_state_output_port(),
                      controller->get_input_port_estimated_state());
      builder.Connect(controller->get_output_port_control(),
                      plant.get_actuation_input_port(panda));
    } else if (config.controller_config.type == "inverse_dynamics") {
      auto controller = builder.AddSystem<InverseDynamicsController>(
          control_plant, Kp, Ki, Kd, false);
      builder.Connect(nominal_state_source->get_output_port(),
                      controller->get_input_port_desired_state());
      builder.Connect(plant.get_state_output_port(robot),
                      controller->get_input_port_estimated_state());
      builder.Connect(controller->get_output_port_control(),
                      plant.get_actuation_input_port());
    } else if (config.controller_config.type == "joint_stiffness") {
      DRAKE_THROW_UNLESS(config.controller_config.ki == 0.0);
      auto controller =
          builder.AddSystem<JointStiffnessController>(control_plant, Kp, Kd);
      builder.Connect(nominal_state_source->get_output_port(),
                      controller->get_input_port_desired_state());
      builder.Connect(plant.get_state_output_port(robot),
                      controller->get_input_port_estimated_state());
      builder.Connect(controller->get_output_port_actuation(),
                      plant.get_actuation_input_port());
    } else {
      throw std::logic_error(
          "Unknown controller, options are 'pid', 'inverse_dynamics', and "
          "'joint_stiffness'");
    }
  }

  auto diagram = builder.Build();

  // Set up the simulator with the specified integration scheme.
  Simulator<double> simulator(*diagram);
  ApplySimulatorConfig(config.simulator_config, &simulator);
  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  if (config.simulator_config.integration_scheme == "cenic") {
    auto& ci = dynamic_cast<CenicIntegrator<double>&>(integrator);
    ci.SetSolverParameters(config.icf_solver_config);
  }

  // Set initial condition. The gripper is open a bit wider than in q_nom.
  VectorXd q_init = q_nom;
  q_init(1) = 0.0;
  q_init(2) = 0.0;
  q_init(7) = 0.035;
  q_init(8) = 0.035;
  plant.SetPositions(&diagram->GetMutableSubsystemContext(
                         plant, &simulator.get_mutable_context()),
                     q_init);

  simulator.Initialize();
  if (config.visualization_config.has_value()) {
    fmt::print("Press any key to continue ...\n");
    getchar();
  }

  const double recording_frames_per_second =
      config.plant_config.time_step == 0 ? 32.0
                                         : 1.0 / config.plant_config.time_step;
  meshcat->StartRecording(recording_frames_per_second);
  using clock = std::chrono::steady_clock;
  clock::time_point sim_start_time = clock::now();
  simulator.AdvanceTo(config.simulation_time);
  clock::time_point sim_end_time = clock::now();
  const double sim_time =
      std::chrono::duration<double>(sim_end_time - sim_start_time).count();
  fmt::print("AdvanceTo() time [sec]: {}\n", sim_time);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  PrintSimulatorStatistics(simulator);

  // Wait for user input again b/c meshcat is too slow to publish the recording.
  if (config.visualization_config.has_value()) {
    fmt::print("Press any key to quit ...\n");
    getchar();
  }

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("\nSimulation of a franka arm with PID control.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
