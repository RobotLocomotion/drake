#include "sim/common/build_iiwa_control.h"

#include <limits>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/first_order_low_pass_filter.h"
#include "drake/systems/primitives/gain.h"

namespace anzu {
namespace sim {

using drake::lcm::DrakeLcmInterface;
using drake::manipulation::kuka_iiwa::kIiwaLcmStatusPeriod;
using drake::manipulation::kuka_iiwa::IiwaCommandReceiver;
using drake::manipulation::kuka_iiwa::IiwaStatusSender;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Adder;
using drake::systems::Context;
using drake::systems::Demultiplexer;
using drake::systems::Gain;
using drake::systems::StateInterpolatorWithDiscreteDerivative;
using drake::systems::System;
using drake::systems::controllers::InverseDynamicsController;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

void BuildIiwaControl(
    const MultibodyPlant<double>& plant,
    const drake::multibody::ModelInstanceIndex iiwa_instance,
    const MultibodyPlant<double>& controller_plant,
    DrakeLcmInterface* lcm, drake::systems::DiagramBuilder<double>* builder,
    double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_kp_gains) {
  const IiwaControlPorts iiwa_control_ports = BuildSimplifiedIiwaControl(
      plant, iiwa_instance, controller_plant, builder, ext_joint_filter_tau,
      desired_kp_gains, true /* enable_feedforward_torque*/);

  const int num_iiwa_positions = controller_plant.num_positions();

  // Create the iiwa command subscriber to receive desired state commands.
  auto iiwa_command_sub = builder->AddSystem(
      LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  iiwa_command_sub->set_name(
      plant.GetModelInstanceName(iiwa_instance) + "_iiwa_command_subscriber");
  auto iiwa_command_receiver =
      builder->AddSystem<IiwaCommandReceiver>(num_iiwa_positions);
  builder->Connect(iiwa_command_sub->get_output_port(),
                   iiwa_command_receiver->get_message_input_port());
  builder->Connect(iiwa_command_receiver->get_commanded_position_output_port(),
                   *iiwa_control_ports.commanded_positions);

  // Add in feedforward torque.
  builder->Connect(iiwa_command_receiver->get_commanded_torque_output_port(),
                   *iiwa_control_ports.commanded_feedforward_torque);

  // Create an iiwa state sender.
  auto iiwa_state_measured_demux =
      builder->AddSystem<Demultiplexer>(
          2 * num_iiwa_positions, num_iiwa_positions);
  auto iiwa_status_pub =
      builder->AddSystem(LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, kIiwaLcmStatusPeriod));
  iiwa_status_pub->set_name(
      plant.GetModelInstanceName(iiwa_instance) + "_iiwa_status_publisher");
  auto iiwa_status_sender =
      builder->AddSystem<IiwaStatusSender>(num_iiwa_positions);
  builder->Connect(plant.get_state_output_port(iiwa_instance),
                   iiwa_state_measured_demux->get_input_port(0));
  builder->Connect(iiwa_state_measured_demux->get_output_port(0),
                   iiwa_status_sender->get_position_measured_input_port());
  builder->Connect(iiwa_state_measured_demux->get_output_port(1),
                   iiwa_status_sender->get_velocity_estimated_input_port());
  builder->Connect(iiwa_status_sender->get_output_port(),
                   iiwa_status_pub->get_input_port());

  // Use the current position until the first command is received.
  builder->Connect(iiwa_state_measured_demux->get_output_port(0),
                   iiwa_command_receiver->get_position_measured_input_port());

  // Also send commanded state through the Iiwa status sender.
  builder->Connect(iiwa_command_receiver->get_commanded_position_output_port(),
                   iiwa_status_sender->get_position_commanded_input_port());

  // Also send control torque through the Iiwa status sender.
  builder->Connect(*iiwa_control_ports.joint_torque,
                   iiwa_status_sender->get_torque_commanded_input_port());

  // TODO(amcastro-tri): is this what we want to send as the "measured
  // torque"? why coming from the controllers instead of from the plant?
  builder->Connect(*iiwa_control_ports.joint_torque,
                   iiwa_status_sender->get_torque_measured_input_port());

  builder->Connect(
      *iiwa_control_ports.external_torque,
      iiwa_status_sender->get_torque_external_input_port());
}

IiwaControlPorts BuildSimplifiedIiwaControl(
    const MultibodyPlant<double>& plant,
    const drake::multibody::ModelInstanceIndex iiwa_instance,
    const MultibodyPlant<double>& controller_plant,
    drake::systems::DiagramBuilder<double>* builder,
    double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_kp_gains,
    bool enable_feedforward_torque) {
  IiwaControlPorts ports{};
  const int num_iiwa_positions = controller_plant.num_positions();
  DRAKE_DEMAND(num_iiwa_positions == 7);

  drake::VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  iiwa_kp = drake::VectorX<double>::Constant(num_iiwa_positions, 100);
  if (!desired_kp_gains) {
    // These are taken from the current FRI driver values.
    // TODO(eric.cousineau): These seem like *very* high values for inverse
    // dynamics on a simulated plant. Investigate overshoot from
    // `robot_follow_joint_sequence`, see if it's just a timing issue.
    iiwa_kp << 2000, 1500, 1500, 1500, 1500, 500, 500;
  } else {
    DRAKE_DEMAND(desired_kp_gains.value().size() == 7);
    iiwa_kp = desired_kp_gains.value();
  }
  iiwa_kd.resize(num_iiwa_positions);
  for (int i = 0; i < num_iiwa_positions; i++) {
    // Critical damping gains.
    iiwa_kd[i] = 2 * std::sqrt(iiwa_kp[i]);
  }
  iiwa_ki = drake::VectorX<double>::Constant(num_iiwa_positions, 1);

  auto iiwa_controller = builder->AddSystem<InverseDynamicsController>(
      controller_plant, iiwa_kp, iiwa_ki, iiwa_kd,
      false /* no feedforward acceleration */);

  builder->Connect(plant.get_state_output_port(iiwa_instance),
                   iiwa_controller->get_input_port_estimated_state());

  auto iiwa_commanded_state_interpolator =
      builder->AddSystem<StateInterpolatorWithDiscreteDerivative>(
          num_iiwa_positions, kIiwaLcmStatusPeriod,
          true /* suppress_initial_transient */);
  builder->Connect(iiwa_commanded_state_interpolator->get_output_port(),
                   iiwa_controller->get_input_port_desired_state());

  if (enable_feedforward_torque) {
    auto adder = builder->template AddSystem<Adder>(2, num_iiwa_positions);
    builder->Connect(iiwa_controller->get_output_port_control(),
                     adder->get_input_port(0));
    builder->Connect(adder->get_output_port(),
                     plant.get_actuation_input_port(iiwa_instance));
    ports.commanded_feedforward_torque = &adder->get_input_port(1);
  } else {
    builder->Connect(iiwa_controller->get_output_port_control(),
                     plant.get_actuation_input_port(iiwa_instance));
  }

  auto torque_gain = builder->AddSystem<Gain>(-1, num_iiwa_positions);
  builder->Connect(iiwa_controller->get_output_port_control(),
                   torque_gain->get_input_port());

  // Filter for simulated external torques. Unlike the real robot, external
  // torques in sim are rather noisy (which propagates into the computed
  // external wrench).
  const double kFirstOrderTimeConstant = ext_joint_filter_tau;
  auto external_torque_filter =
      builder->AddSystem<drake::systems::FirstOrderLowPassFilter<double>>(
          kFirstOrderTimeConstant, num_iiwa_positions);
  builder->Connect(
      plant.get_generalized_contact_forces_output_port(iiwa_instance),
      external_torque_filter->get_input_port());

  // Create a running constraint on the external torque limit.  Values taken
  // from a TRI Iiwa station.
  Eigen::VectorXd external_torque_limit(num_iiwa_positions);
  external_torque_limit.fill(std::numeric_limits<double>::infinity());
  external_torque_limit << 100, 100, 100, 100, 50, 30, 30;
  external_torque_filter->AddExternalConstraint(
      drake::systems::ExternalSystemConstraint(
          "external torque limit",
          drake::systems::SystemConstraintBounds(
              external_torque_limit * -1, external_torque_limit),
          [](const System<double>& system,
             const Context<double>& context,
             Eigen::VectorXd* value) {
            *value = system.get_output_port(0).Eval(context);
          }));

  ports.commanded_positions =
      &iiwa_commanded_state_interpolator->get_input_port();
  ports.external_torque = &external_torque_filter->get_output_port();
  ports.joint_torque = &torque_gain->get_output_port();
  return ports;
}

}  // namespace sim
}  // namespace anzu
