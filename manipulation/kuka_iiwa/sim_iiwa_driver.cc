#include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"

#include <string>

#include "drake/common/default_scalars.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/first_order_low_pass_filter.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using systems::Adder;
using systems::Context;
using systems::Demultiplexer;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::FirstOrderLowPassFilter;
using systems::PassThrough;
using systems::StateInterpolatorWithDiscreteDerivative;
using systems::System;
using systems::controllers::InverseDynamics;
using systems::controllers::InverseDynamicsController;

namespace {

/* The return type of MakeInverseDynamicsGains(), immediately below. */
struct Gains {
  VectorX<double> kp;
  VectorX<double> ki;
  VectorX<double> kd;
};

/* Given the user-provided `kp` gains (if any), return all PID gains. */
Gains MakeInverseDynamicsGains(const std::optional<Eigen::VectorXd>& kp) {
  Gains result;

  // The default values are taken from the current FRI driver.
  // TODO(EricCousineau-TRI): These seem like *very* high values for inverse
  // dynamics on a simulated plant. Investigate overshoot from
  // `robot_follow_joint_sequence`, see if it's just a timing issue.
  result.kp =
      kp.value_or((Eigen::VectorXd(7) << 2000, 1500, 1500, 1500, 1500, 500, 500)
                      .finished());

  // Critical damping gains.
  result.kd = 2 * result.kp.array().sqrt();

  result.ki = VectorX<double>::Constant(result.kp.size(), 1.0);

  return result;
}

}  // namespace

template <typename T>
SimIiwaDriver<T>::SimIiwaDriver(
    const IiwaControlMode control_mode,
    const multibody::MultibodyPlant<T>* const controller_plant,
    const double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& kp_gains)
    : Diagram<T>(systems::SystemTypeTag<SimIiwaDriver>{}) {
  DRAKE_THROW_UNLESS(controller_plant != nullptr);
  const int num_positions = controller_plant->num_positions();

  DiagramBuilder<T> builder;

  // Declare the `state` input port to accept MbP's {i'th}_state output port,
  // and demux it into separate positions and velocities.
  auto state_demux = builder.template AddNamedSystem<Demultiplexer>(
      "demultiplexer", 2 * num_positions, num_positions);
  builder.ExportInput(state_demux->get_input_port(), "state");

  // Declare the `generalized_contact_forces` input port to accept the MbP's
  // {i'th}_generalized_contact_forces output port. Filter this because unlike
  // the real robot external torques in sim are rather noisy (which propagates
  // into the computed external wrench).
  auto contact_forces =
      builder.template AddNamedSystem<FirstOrderLowPassFilter>(
          "low_pass_filter", ext_joint_filter_tau, num_positions);
  builder.ExportInput(contact_forces->get_input_port(),
                      "generalized_contact_forces");

  // When position control is enabled, declare the `position` input port,
  // interpolate position commands into state commands, and then use a full
  // inverse dynamics controller. Otherwise, use inverse dynamics only for
  // gravity compensation.
  const System<T>* inverse_dynamics{};
  if (position_enabled(control_mode)) {
    using StateInterpolator = StateInterpolatorWithDiscreteDerivative<T>;
    auto interpolator = builder.template AddNamedSystem<StateInterpolator>(
        "velocity_interpolator", num_positions, kIiwaLcmStatusPeriod,
        true /* suppress initial transient */);
    builder.ExportInput(interpolator->get_input_port(), "position");
    const Gains inverse_dynamics_gains = MakeInverseDynamicsGains(kp_gains);
    inverse_dynamics =
        builder.template AddNamedSystem<InverseDynamicsController>(
            "inverse_dynamics_controller", *controller_plant,
            inverse_dynamics_gains.kp, inverse_dynamics_gains.ki,
            inverse_dynamics_gains.kd, false /* no feedforward acceleration */);
    builder.Connect(interpolator->GetOutputPort("state"),
                    inverse_dynamics->GetInputPort("desired_state"));
  } else {
    inverse_dynamics = builder.template AddNamedSystem<InverseDynamics>(
        "gravity_compensation", controller_plant,
        InverseDynamics<T>::InverseDynamicsMode::kGravityCompensation);
  }
  builder.ConnectInput("state",
                       inverse_dynamics->GetInputPort("estimated_state"));

  // When torque control is enabled, declare the `torque` input port and add it
  // to the inverse dynamics output. Otherwise, use the inverse dynamics output
  // by itself.
  const System<T>* actuation = nullptr;
  if (torque_enabled(control_mode)) {
    actuation = builder.template AddNamedSystem<Adder>("+", 2, num_positions);
    builder.Connect(inverse_dynamics->GetOutputPort("generalized_force"),
                    actuation->get_input_port(0));
    builder.ExportInput(actuation->get_input_port(1), "torque");
  } else {
    actuation = inverse_dynamics;
  }

  // Declare the various output ports.
  builder.ExportOutput(actuation->get_output_port(), "actuation");
  if (position_enabled(control_mode)) {
    auto pass = builder.template AddNamedSystem<PassThrough>(
        "position_pass_through", num_positions);
    builder.ConnectInput("position", pass->get_input_port());
    builder.ExportOutput(pass->get_output_port(), "position_commanded");
  } else {
    builder.ExportOutput(state_demux->get_output_port(0), "position_commanded");
  }
  builder.ExportOutput(state_demux->get_output_port(0), "position_measured");
  builder.ExportOutput(state_demux->get_output_port(1), "velocity_estimated");
  {
    auto pass = builder.template AddNamedSystem<PassThrough>(
        "state_pass_through", 2 * num_positions);
    builder.ConnectInput("state", pass->get_input_port());
    builder.ExportOutput(pass->get_output_port(), "state_estimated");
  }
  builder.ExportOutput(actuation->get_output_port(), "torque_commanded");
  // TODO(amcastro-tri): is this what we want to send as the "measured
  // torque"? why coming from the controllers instead of from the plant?
  builder.ExportOutput(actuation->get_output_port(), "torque_measured");
  builder.ExportOutput(contact_forces->get_output_port(), "torque_external");

  builder.BuildInto(this);
}

template <typename T>
template <typename U>
SimIiwaDriver<T>::SimIiwaDriver(const SimIiwaDriver<U>& other)
    : Diagram<T>(systems::SystemTypeTag<SimIiwaDriver>{}, other) {}

template <typename T>
const System<double>& SimIiwaDriver<T>::AddToBuilder(
    DiagramBuilder<double>* builder, const MultibodyPlant<double>& plant,
    const ModelInstanceIndex iiwa_instance,
    const MultibodyPlant<double>& controller_plant, double ext_joint_filter_tau,
    const std::optional<Eigen::VectorXd>& desired_iiwa_kp_gains,
    IiwaControlMode control_mode) {
  const std::string name =
      fmt::format("IiwaDriver({})", plant.GetModelInstanceName(iiwa_instance));
  auto system = builder->AddNamedSystem<SimIiwaDriver<double>>(
      name, control_mode, &controller_plant, ext_joint_filter_tau,
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

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::manipulation::kuka_iiwa::SimIiwaDriver);
