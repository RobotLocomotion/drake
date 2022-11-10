#include "drake/manipulation/kuka_iiwa/build_iiwa_control.h"

#include <limits>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/first_order_low_pass_filter.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using drake::lcm::DrakeLcmInterface;
using manipulation::kuka_iiwa::IiwaCommandReceiver;
using manipulation::kuka_iiwa::IiwaStatusSender;
using manipulation::kuka_iiwa::kIiwaLcmStatusPeriod;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using systems::Adder;
using systems::Context;
using systems::Demultiplexer;
using systems::Gain;
using systems::PassThrough;
using systems::StateInterpolatorWithDiscreteDerivative;
using systems::System;
using systems::controllers::InverseDynamics;
using systems::controllers::InverseDynamicsController;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::LcmSubscriberSystem;

using InverseDynamicsMode = InverseDynamics<double>::InverseDynamicsMode;

void BuildIiwaControl(const MultibodyPlant<double>& plant,
                      const multibody::ModelInstanceIndex iiwa_instance,
                      const MultibodyPlant<double>& controller_plant,
                      DrakeLcmInterface* lcm,
                      systems::DiagramBuilder<double>* builder,
                      double ext_joint_filter_tau,
                      const std::optional<Eigen::VectorXd>& desired_kp_gains,
                      IiwaControlMode control_mode) {
  const IiwaControlPorts iiwa_control_ports = BuildSimplifiedIiwaControl(
      plant, iiwa_instance, controller_plant, builder, ext_joint_filter_tau,
      desired_kp_gains, control_mode);

  const int num_iiwa_positions = controller_plant.num_positions();

  // Create the Iiwa command subscriber to receive desired state commands.
  auto iiwa_command_sub = builder->AddSystem(
      LcmSubscriberSystem::Make<lcmt_iiwa_command>("IIWA_COMMAND", lcm));
  iiwa_command_sub->set_name(
      plant.GetModelInstanceName(iiwa_instance) + "_iiwa_command_subscriber");
  auto iiwa_command_receiver =
      builder->AddSystem<IiwaCommandReceiver>(num_iiwa_positions, control_mode);
  builder->Connect(iiwa_command_sub->get_output_port(),
                   iiwa_command_receiver->get_message_input_port());

  const bool has_position =
      static_cast<bool>(control_mode & IiwaControlMode::kPosition);
  const bool has_torque =
      static_cast<bool>(control_mode & IiwaControlMode::kTorque);

  // Connect desired positions.
  if (has_position) {
      builder->Connect(
          iiwa_command_receiver->get_commanded_position_output_port(),
          *iiwa_control_ports.commanded_positions);
  }

  // Connect desired torque.
  if (has_torque) {
    builder->Connect(
        iiwa_command_receiver->get_commanded_torque_output_port(),
        *iiwa_control_ports.commanded_torque);
  }

  // Create an Iiwa state sender.
  auto iiwa_state_measured_demux =
      builder->AddSystem<Demultiplexer>(
          2 * num_iiwa_positions, num_iiwa_positions);
  auto iiwa_status_pub =
      builder->AddSystem(LcmPublisherSystem::Make<lcmt_iiwa_status>(
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
  if (has_position) {
    builder->Connect(
        iiwa_command_receiver->get_commanded_position_output_port(),
        iiwa_status_sender->get_position_commanded_input_port());
  } else {
    // If we don't supply positions, simply loopback the estimated states.
    builder->Connect(
        iiwa_state_measured_demux->get_output_port(0),
        iiwa_status_sender->get_position_commanded_input_port());
  }

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
    const multibody::ModelInstanceIndex iiwa_instance,
    const MultibodyPlant<double>& controller_plant,
    systems::DiagramBuilder<double>* builder, double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_kp_gains,
    IiwaControlMode control_mode) {
  DRAKE_DEMAND(IsValid(control_mode));

  const bool has_position =
      static_cast<bool>(control_mode & IiwaControlMode::kPosition);
  const bool has_torque =
      static_cast<bool>(control_mode & IiwaControlMode::kTorque);

  IiwaControlPorts ports{};
  const int num_iiwa_positions = controller_plant.num_positions();
  DRAKE_THROW_UNLESS(num_iiwa_positions == 7);

  // Intercept desired torque so we can also send it as measured torque.
  const PassThrough<double>* torque_proxy =
      builder->AddSystem<PassThrough>(num_iiwa_positions);
  builder->Connect(
      torque_proxy->get_output_port(),
      plant.get_actuation_input_port(iiwa_instance));

  if (has_position) {
    VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;

    // The default values are taken from the current FRI driver.
    // TODO(EricCousineau-TRI): These seem like *very* high values for inverse
    // dynamics on a simulated plant. Investigate overshoot from
    // `robot_follow_joint_sequence`, see if it's just a timing issue.
    iiwa_kp = desired_kp_gains.value_or((Eigen::VectorXd(7)
        << 2000, 1500, 1500, 1500, 1500, 500, 500).finished());
    DRAKE_THROW_UNLESS(iiwa_kp.size() == 7);

    iiwa_kd.resize(num_iiwa_positions);
    for (int i = 0; i < num_iiwa_positions; ++i) {
      // Critical damping gains.
      iiwa_kd[i] = 2 * std::sqrt(iiwa_kp[i]);
    }
    iiwa_ki = VectorX<double>::Constant(num_iiwa_positions, 1);

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

    ports.commanded_positions =
        &iiwa_commanded_state_interpolator->get_input_port();
    if (has_torque) {
      // Optional feedforward torque.
      auto adder = builder->template AddSystem<Adder>(2, num_iiwa_positions);
      builder->Connect(iiwa_controller->get_output_port_control(),
                       adder->get_input_port(0));
      builder->Connect(adder->get_output_port(),
                       torque_proxy->get_input_port());
      ports.commanded_torque = &adder->get_input_port(1);
    } else {
      builder->Connect(iiwa_controller->get_output_port_control(),
                       torque_proxy->get_input_port());
    }
  } else if (has_torque) {
    DRAKE_THROW_UNLESS(!desired_kp_gains.has_value());
    // Torque alone, added to gravity compensation.
    auto gravity_comp = builder->AddSystem<InverseDynamics>(
        &controller_plant, InverseDynamicsMode::kGravityCompensation);
    builder->Connect(
        plant.get_state_output_port(iiwa_instance),
        gravity_comp->get_input_port_estimated_state());
    auto adder = builder->template AddSystem<Adder>(2, num_iiwa_positions);
    builder->Connect(
        gravity_comp->get_output_port_force(),
        adder->get_input_port(0));
    ports.commanded_torque = &adder->get_input_port(1);
    builder->Connect(
        adder->get_output_port(),
        torque_proxy->get_input_port());
  }

  // Filter for simulated external torques. Unlike the real robot, external
  // torques in sim are rather noisy (which propagates into the computed
  // external wrench).
  const double kFirstOrderTimeConstant = ext_joint_filter_tau;
  auto external_torque_filter =
      builder->AddSystem<systems::FirstOrderLowPassFilter<double>>(
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
      systems::ExternalSystemConstraint(
          "external torque limit",
          systems::SystemConstraintBounds(external_torque_limit * -1,
                                          external_torque_limit),
          [](const System<double>& system, const Context<double>& context,
             Eigen::VectorXd* value) {
            // TODO(jwnimmer-tri) Add a unit test for this.
            *value = system.get_output_port(0).Eval(context);
          }));

  // TODO(eric.cousineau): Why do we flip this?
  auto torque_gain = builder->AddSystem<Gain>(-1, num_iiwa_positions);
  builder->Connect(torque_proxy->get_output_port(),
                   torque_gain->get_input_port());

  ports.external_torque = &external_torque_filter->get_output_port();
  ports.joint_torque = &torque_gain->get_output_port();
  return ports;
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
