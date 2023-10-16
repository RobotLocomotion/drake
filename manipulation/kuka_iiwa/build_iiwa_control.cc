#include "drake/manipulation/kuka_iiwa/build_iiwa_control.h"

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/gain.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using drake::lcm::DrakeLcmInterface;
using kuka_iiwa::internal::SimIiwaDriver;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using systems::Demultiplexer;
using systems::DiagramBuilder;
using systems::Gain;
using systems::System;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::LcmSubscriberSystem;

void BuildIiwaControl(
    const MultibodyPlant<double>& plant, const ModelInstanceIndex iiwa_instance,
    const MultibodyPlant<double>& controller_plant, DrakeLcmInterface* lcm,
    DiagramBuilder<double>* builder, double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains,
    IiwaControlMode control_mode) {
  const IiwaControlPorts sim_ports = BuildSimplifiedIiwaControl(
      plant, iiwa_instance, controller_plant, builder, ext_joint_filter_tau,
      desired_iiwa_kp_gains, control_mode);

  const int num_iiwa_positions = controller_plant.num_positions();
  const std::string model_name = plant.GetModelInstanceName(iiwa_instance);

  // Create the Iiwa command receiver.
  auto command_sub = builder->AddNamedSystem(
      fmt::format("{}_iiwa_command_subscriber", model_name),
      LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND", lcm));
  auto command_decode = builder->AddNamedSystem<IiwaCommandReceiver>(
      fmt::format("{}_iiwa_command_receiver", model_name), num_iiwa_positions,
      control_mode);
  builder->Connect(command_sub->get_output_port(),
                   command_decode->get_message_input_port());

  // Use the current position until the first command is received. Note that we
  // connect the position directly from the plant, not the (possibly sampled or
  // delayed) `sim_port.position_measured` output of the driver.
  auto state_demux = builder->template AddNamedSystem<Demultiplexer>(
      fmt::format("{}_iiwa_state_demultiplexer", model_name),
      2 * num_iiwa_positions, num_iiwa_positions);
  builder->Connect(plant.get_state_output_port(iiwa_instance),
                   state_demux->get_input_port());
  builder->Connect(state_demux->get_output_port(0),
                   command_decode->get_position_measured_input_port());

  // Connect receiver output to sim controller input.
  if (position_enabled(control_mode)) {
    builder->Connect(command_decode->get_commanded_position_output_port(),
                     *sim_ports.commanded_positions);
  }
  if (torque_enabled(control_mode)) {
    builder->Connect(command_decode->get_commanded_torque_output_port(),
                     *sim_ports.commanded_torque);
  }

  // Create an Iiwa status sender.
  auto status_pub = builder->AddNamedSystem(
      fmt::format("{}_iiwa_status_publisher", model_name),
      LcmPublisherSystem::Make<lcmt_iiwa_status>("IIWA_STATUS", lcm,
                                                 kIiwaLcmStatusPeriod));
  auto status_encode = builder->AddNamedSystem<IiwaStatusSender>(
      fmt::format("{}_iiwa_status_sender", model_name), num_iiwa_positions);
  builder->Connect(status_encode->get_output_port(),
                   status_pub->get_input_port());

  // Connect sim controller output to status input.
  builder->Connect(*sim_ports.position_commanded,
                   status_encode->get_position_commanded_input_port());
  builder->Connect(*sim_ports.position_measured,
                   status_encode->get_position_measured_input_port());
  builder->Connect(*sim_ports.velocity_estimated,
                   status_encode->get_velocity_estimated_input_port());
  builder->Connect(*sim_ports.joint_torque,
                   status_encode->get_torque_commanded_input_port());
  builder->Connect(*sim_ports.torque_measured,
                   status_encode->get_torque_measured_input_port());
  builder->Connect(*sim_ports.external_torque,
                   status_encode->get_torque_external_input_port());
}

IiwaControlPorts BuildSimplifiedIiwaControl(
    const MultibodyPlant<double>& plant, const ModelInstanceIndex iiwa_instance,
    const MultibodyPlant<double>& controller_plant,
    DiagramBuilder<double>* builder, double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains,
    IiwaControlMode control_mode) {
  const int num_positions = controller_plant.num_positions();
  DRAKE_THROW_UNLESS(num_positions == 7);

  // Add the sim driver to the builder.
  const System<double>* const system = &internal::AddSimIiwaDriver(
      plant, iiwa_instance, controller_plant, builder, ext_joint_filter_tau,
      desired_iiwa_kp_gains, control_mode);

  // Return the necessary port pointers.
  IiwaControlPorts result;
  if (position_enabled(control_mode)) {
    result.commanded_positions = &system->GetInputPort("position");
  }
  if (torque_enabled(control_mode)) {
    result.commanded_torque = &system->GetInputPort("torque");
  }
  result.position_commanded = &system->GetOutputPort("position_commanded");
  result.position_measured = &system->GetOutputPort("position_measured");
  result.velocity_estimated = &system->GetOutputPort("velocity_estimated");
  {
    // TODO(eric.cousineau): Why do we flip this?
    auto negate = builder->AddNamedSystem<Gain>(
        fmt::format("sign_flip_{}_torque_commanded",
                    plant.GetModelInstanceName(iiwa_instance)),
        -1, num_positions);
    builder->Connect(system->GetOutputPort("torque_commanded"),
                     negate->get_input_port());
    result.joint_torque = &negate->get_output_port();
  }
  {
    // TODO(eric.cousineau): Why do we flip this?
    auto negate = builder->AddNamedSystem<Gain>(
        fmt::format("sign_flip_{}_torque_measured",
                    plant.GetModelInstanceName(iiwa_instance)),
        -1, num_positions);
    builder->Connect(system->GetOutputPort("torque_measured"),
                     negate->get_input_port());
    result.torque_measured = &negate->get_output_port();
  }
  result.external_torque = &system->GetOutputPort("torque_external");
  return result;
}

namespace internal {

const System<double>& AddSimIiwaDriver(
    const MultibodyPlant<double>& plant, const ModelInstanceIndex iiwa_instance,
    const MultibodyPlant<double>& controller_plant,
    DiagramBuilder<double>* builder, double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains,
    IiwaControlMode control_mode) {
  const std::string inner_name =
      fmt::format("IiwaDriver({})", plant.GetModelInstanceName(iiwa_instance));
  auto system = builder->template AddNamedSystem<SimIiwaDriver>(
      inner_name, control_mode, &controller_plant, ext_joint_filter_tau,
      desired_iiwa_kp_gains);
  builder->Connect(plant.get_state_output_port(iiwa_instance),
                   system->GetInputPort("state"));
  builder->Connect(
      plant.get_generalized_contact_forces_output_port(iiwa_instance),
      system->GetInputPort("generalized_contact_forces"));
  builder->Connect(system->GetOutputPort("actuation"),
                   plant.get_actuation_input_port(iiwa_instance));
  return *system;
}

}  // namespace internal
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
