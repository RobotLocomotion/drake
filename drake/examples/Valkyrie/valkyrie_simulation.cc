#include <memory>

#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/common/text_logging.h"
#include "drake/examples/Valkyrie/actuator_effort_to_rigid_body_plant_input_converter.h"
#include "drake/examples/Valkyrie/robot_command_to_desired_effort_converter.h"
#include "drake/examples/Valkyrie/robot_state_encoder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/pass_through.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

namespace drake {
namespace systems {

using std::move;
using std::unique_ptr;
using std::make_unique;
using std::map;

using bot_core::atlas_command_t;
using bot_core::robot_state_t;

using drake::lcm::DrakeLcm;
using lcm::LcmSubscriberSystem;
using lcm::LcmPublisherSystem;
using multibody::joints::kRollPitchYaw;

// TODO(tkoolen) shouldn't hard-code. Copied from fixed point file for Valkyrie.
// Need C++ equivalent of resolveConstraints.
VectorX<double> RPYValkyrieFixedPointState() {
  VectorX<double> ret(72);
  ret << 0, 0, 1.025, 0, 0, 0, 0, 0, 0, 0, 0.300196631343025, 1.25, 0,
      0.785398163397448, 1.571, 0, 0, 0.300196631343025, -1.25, 0,
      -0.785398163397448, 1.571, 0, 0, 0, 0, -0.49, 1.205, -0.71, 0, 0, 0,
      -0.49, 1.205, -0.71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  return ret;
}

int main(int argc, const char** argv) {
  //  drake::log()->set_level(spdlog::level::trace);
  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      kRollPitchYaw, nullptr /* weld to frame */, tree_ptr.get());
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr));
  // Contact parameters set arbitrarily.
  plant.set_contact_parameters(10000., 100., 10.);
  const auto& tree = plant.get_rigid_body_tree();

  // RigidBodyActuators.
  std::vector<const RigidBodyActuator*> actuators;
  for (const auto& actuator : tree.actuators) {
    actuators.push_back(&actuator);
  }
  // Currently, all RigidBodyActuators are assumed to be one-dimensional.
  const int actuator_effort_length = 1;

  // LCM communication.
  DrakeLcm lcm;

  // LCM inputs.
  auto& atlas_command_subscriber = *builder.AddSystem(
      LcmSubscriberSystem::Make<atlas_command_t>("ROBOT_COMMAND", &lcm));
  auto& robot_command_to_desired_effort_converter =
      *builder.AddSystem<RobotCommandToDesiredEffortConverter>(actuators);

  // Placeholder for actuator dynamics.
  map<const RigidBodyActuator*, System<double>*> actuator_dynamics;
  for (const auto& actuator : actuators) {
    actuator_dynamics.emplace(std::make_pair(
        actuator,
        builder.AddSystem<PassThrough<double>>(actuator_effort_length)));
  }

  // Conversion from desired efforts to RigidBodyPlant input vector.
  auto& actuator_effort_to_rigid_body_plant_input_converter =
      *builder.AddSystem<ActuatorEffortToRigidBodyPlantInputConverter>(
          actuators);

  // Placeholder for effort sensors.
  map<const RigidBodyActuator*, System<double>*> effort_sensors;
  for (const auto& actuator : actuators) {
    effort_sensors.emplace(std::make_pair(
        actuator,
        builder.AddSystem<PassThrough<double>>(actuator_effort_length)));
  }

  // LCM outputs.
  auto& robot_state_encoder =
      *builder.AddSystem<RobotStateEncoder>(plant.get_rigid_body_tree());
  auto& robot_state_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<robot_state_t>("EST_ROBOT_STATE", &lcm));

  // TODO(tkoolen): Force/torque sensors. Zero for now.
  map<Side, System<double>*> hand_wrench_sensors;
  map<Side, System<double>*> foot_wrench_sensors;
  for (Side side : Side::values) {
    hand_wrench_sensors[side] =
        builder.AddSystem<ConstantVectorSource>(Vector6<double>::Zero().eval());
    foot_wrench_sensors[side] =
        builder.AddSystem<ConstantVectorSource>(Vector6<double>::Zero().eval());
  }

  // Visualizer.
  const DrakeVisualizer& visualizer_publisher =
      *builder.template AddSystem<DrakeVisualizer>(tree, &lcm);

  // Connections.
  // LCM message to desired effort conversion.
  builder.Connect(atlas_command_subscriber,
                  robot_command_to_desired_effort_converter);

  for (const auto& actuator : actuators) {
    // Desired effort inputs to actuator dynamics.
    auto desired_effort_output =
        robot_command_to_desired_effort_converter.desired_effort_output_port(
            *actuator);
    auto desired_effort_input =
        actuator_dynamics.at(actuator)->get_input_port(0);
    builder.Connect(desired_effort_output, desired_effort_input);

    // Efforts to effort sensors.
    const auto& effort_output_port =
        actuator_dynamics.at(actuator)->get_output_port(0);
    const auto& measured_effort_input_port =
        effort_sensors.at(actuator)->get_input_port(0);
    builder.Connect(effort_output_port, measured_effort_input_port);

    // Efforts to rigid body plant input
    builder.Connect(
        effort_output_port,
        actuator_effort_to_rigid_body_plant_input_converter.effort_input_port(
            *actuator));

    // Effort sensors to robot state encoder.
    const auto& measured_effort_output_port =
        effort_sensors.at(actuator)->get_output_port(0);
    const auto& state_encoder_effort_input_port =
        robot_state_encoder.effort_port(*actuator);
    builder.Connect(measured_effort_output_port,
                    state_encoder_effort_input_port);
  }

  // Plant input to plant.
  builder.Connect(actuator_effort_to_rigid_body_plant_input_converter, plant);

  // Raw state vector to visualizer.
  builder.Connect(plant.state_output_port(),
                  visualizer_publisher.get_input_port(0));

  // Kinematics results to robot state encoder.
  builder.Connect(plant.kinematics_results_output_port(),
                  robot_state_encoder.kinematics_results_port());

  for (Side side : Side::values) {
    builder.Connect(hand_wrench_sensors.at(side)->get_output_port(0),
                    robot_state_encoder.hand_contact_wrench_port(side));
    builder.Connect(foot_wrench_sensors.at(side)->get_output_port(0),
                    robot_state_encoder.foot_contact_wrench_port(side));
  }

  // Robot state encoder to robot state publisher.
  builder.Connect(robot_state_encoder, robot_state_publisher);

  auto diagram = builder.Build();

  // Create simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram);
  auto context = simulator->get_mutable_context();
  // Integrator set arbitrarily.
  simulator->reset_integrator<ExplicitEulerIntegrator<double>>(*diagram, 1e-3,
                                                               context);

  // Set initial state.
  auto plant_context = diagram->GetMutableSubsystemContext(context, &plant);

  // TODO(tkoolen): make it easy to specify a different initial configuration.
  VectorX<double> initial_state = RPYValkyrieFixedPointState();
  plant.set_state_vector(plant_context, initial_state);
  lcm.StartReceiveThread();

  while (true) {
    const double time = context->get_time();
    SPDLOG_TRACE(drake::log(), "Time is now {}", time);
    simulator->StepTo(time + 0.01);
  }
}

}  // namespace systems
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::systems::main(argc, argv);
}
