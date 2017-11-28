#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/examples/valkyrie/actuator_effort_to_rigid_body_plant_input_converter.h"
#include "drake/examples/valkyrie/robot_command_to_desired_effort_converter.h"
#include "drake/examples/valkyrie/robot_state_encoder.h"
#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace examples {
namespace valkyrie {

class ValkyrieSimulationDiagram : public systems::Diagram<double> {
 public:
  explicit ValkyrieSimulationDiagram(lcm::DrakeLcm* lcm) {
    systems::DiagramBuilder<double> builder;

    // Create RigidBodyTree.
    auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        FindResourceOrThrow(
            "drake/examples/valkyrie/urdf/urdf/"
            "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"),
        multibody::joints::kRollPitchYaw, nullptr /* weld to frame */,
        tree_ptr.get());
    multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

    // Instantiate a RigidBodyPlant from the RigidBodyTree.
    plant_ =
        builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree_ptr));
    plant_->set_name("plant");

    // Contact parameters
    const double kYoungsModulus = 1e8;  // Pa
    const double kDissipation = 5.0;  // s/m
    const double kStaticFriction = 0.9;
    const double kDynamicFriction = 0.5;
    systems::CompliantMaterial default_material;
    default_material.set_youngs_modulus(kYoungsModulus)
        .set_dissipation(kDissipation)
        .set_friction(kStaticFriction, kDynamicFriction);
    plant_->set_default_compliant_material(default_material);

    const double kStictionSlipTolerance = 0.01;  // m/s
    const double kContactArea = 2e-3;  // m^2
    systems::CompliantContactModelParameters model_parameters;
    model_parameters.characteristic_area = kContactArea;
    model_parameters.v_stiction_tolerance = kStictionSlipTolerance;
    plant_->set_contact_model_parameters(model_parameters);

    const auto& tree = plant_->get_rigid_body_tree();

    // RigidBodyActuators.
    std::vector<const RigidBodyActuator*> actuators;
    for (const auto& actuator : tree.actuators) {
      actuators.push_back(&actuator);
    }
    // Currently, all RigidBodyActuators are assumed to be one-dimensional.
    const int actuator_effort_length = 1;

    // LCM inputs.
    auto& atlas_command_subscriber = *builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<bot_core::atlas_command_t>(
            "ROBOT_COMMAND", lcm));
    atlas_command_subscriber.set_name("atlas_command_subscriber");

    auto& robot_command_to_desired_effort_converter =
        *builder.AddSystem<systems::RobotCommandToDesiredEffortConverter>(
            actuators);
    robot_command_to_desired_effort_converter.set_name(
        "robot_command_to_desired_effort_converter");

    // Placeholder for actuator dynamics.
    std::map<const RigidBodyActuator*, System<double>*> actuator_dynamics;
    for (const auto& actuator : actuators) {
      auto pass_through_ptr = builder.AddSystem<systems::PassThrough<double>>(
          actuator_effort_length);
      pass_through_ptr->set_name(actuator->name_ + "_actuator_dynamics");

      actuator_dynamics.emplace(std::make_pair(actuator, pass_through_ptr));
    }

    // Conversion from desired efforts to RigidBodyPlant input vector.
    auto& actuator_effort_to_rigid_body_plant_input_converter =
        *builder
             .AddSystem<systems::ActuatorEffortToRigidBodyPlantInputConverter>(
                 actuators);
    actuator_effort_to_rigid_body_plant_input_converter.set_name(
        "actuator_effort_to_rigid_body_plant_input_converter");

    // Placeholder for effort sensors.
    std::map<const RigidBodyActuator*, System<double>*> effort_sensors;
    for (const auto& actuator : actuators) {
      auto pass_through_ptr = builder.AddSystem<systems::PassThrough<double>>(
          actuator_effort_length);
      pass_through_ptr->set_name(actuator->name_ + "_trq_sensor");

      effort_sensors.emplace(std::make_pair(actuator, pass_through_ptr));
    }

    // LCM outputs.
    std::vector<RigidBodyFrame<double>> force_torque_sensor_info = {
        RigidBodyFrame<double>("leftFootFTSensor", tree.FindBody("leftFoot"),
                               Isometry3<double>::Identity()),
        RigidBodyFrame<double>("rightFootFTSensor", tree.FindBody("rightFoot"),
                               Isometry3<double>::Identity())};

    auto& robot_state_encoder = *builder.AddSystem<systems::RobotStateEncoder>(
        plant_->get_rigid_body_tree(), force_torque_sensor_info);
    robot_state_encoder.set_name("robot_state_encoder");

    auto& robot_state_publisher = *builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<bot_core::robot_state_t>(
            "EST_ROBOT_STATE", lcm));
    robot_state_publisher.set_name("robot_state_publisher");

    // Visualizer.
    systems::DrakeVisualizer& visualizer_publisher =
        *builder.template AddSystem<systems::DrakeVisualizer>(tree, lcm);
    visualizer_publisher.set_name("visualizer_publisher");

    systems::ContactResultsToLcmSystem<double>& contact_viz =
        *builder.template AddSystem<systems::ContactResultsToLcmSystem<double>>(
            tree);
    contact_viz.set_name("contact_viz");

    auto& contact_results_publisher = *builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
            "CONTACT_RESULTS", lcm));
    contact_results_publisher.set_name("contact_results_publisher");

    contact_results_publisher.set_publish_period(1e-3);
    visualizer_publisher.set_publish_period(1e-3);
    robot_state_publisher.set_publish_period(1e-3);

    // Connections.
    // LCM message to desired effort conversion.
    builder.Connect(atlas_command_subscriber,
                    robot_command_to_desired_effort_converter);

    for (const auto& actuator : actuators) {
      // Desired effort inputs to actuator dynamics.
      const auto& desired_effort_output =
          robot_command_to_desired_effort_converter.desired_effort_output_port(
              *actuator);
      const auto& desired_effort_input =
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
    builder.Connect(
        actuator_effort_to_rigid_body_plant_input_converter.get_output_port(0),
        plant_->get_input_port(0));

    // Raw state vector to visualizer.
    builder.Connect(plant_->state_output_port(),
                    visualizer_publisher.get_input_port(0));

    // Kinematics results to robot state encoder.
    builder.Connect(plant_->kinematics_results_output_port(),
                    robot_state_encoder.kinematics_results_port());

    // Contact results to robot state encoder.
    builder.Connect(plant_->contact_results_output_port(),
                    robot_state_encoder.contact_results_port());

    // Contact results to lcm msg.
    builder.Connect(plant_->contact_results_output_port(),
                    contact_viz.get_input_port(0));
    builder.Connect(contact_viz.get_output_port(0),
                    contact_results_publisher.get_input_port(0));

    // Robot state encoder to robot state publisher.
    builder.Connect(robot_state_encoder, robot_state_publisher);

    builder.BuildInto(this);
  }

  systems::RigidBodyPlant<double>* get_mutable_plant() { return plant_; }

 private:
  systems::RigidBodyPlant<double>* plant_;
};

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake
