#include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"

#include <limits>
#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/first_order_low_pass_filter.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/systems/primitives/sparse_matrix_gain.h"

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
    const IiwaDriver& driver_config,
    const multibody::MultibodyPlant<T>* controller_plant)
    : Diagram<T>(systems::SystemTypeTag<SimIiwaDriver>{}) {
  DRAKE_THROW_UNLESS(controller_plant != nullptr);
  const int num_positions = controller_plant->num_positions();
  const IiwaControlMode control_mode =
      ParseIiwaControlMode(driver_config.control_mode);

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
          "low_pass_filter", driver_config.ext_joint_filter_tau, num_positions);
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
        "velocity_interpolator", num_positions, driver_config.lcm_status_period,
        true /* suppress initial transient */);
    builder.ExportInput(interpolator->get_input_port(), "position");
    const Gains inverse_dynamics_gains =
        MakeInverseDynamicsGains(driver_config.desired_kp_gains);
    inverse_dynamics =
        builder.template AddNamedSystem<InverseDynamicsController>(
            "inverse_dynamics_controller", *controller_plant,
            inverse_dynamics_gains.kp, inverse_dynamics_gains.ki,
            inverse_dynamics_gains.kd, false /* no feedforward acceleration */);
    builder.Connect(interpolator->GetOutputPort("state"),
                    inverse_dynamics->GetInputPort("desired_state"));
    const std::vector<int> state_demux_sizes = {// num_positions, num_velocities
                                                num_positions, num_positions};
    auto state_desired_demux =
        builder.template AddSystem<Demultiplexer>(state_demux_sizes);
    builder.Connect(interpolator->get_output_port(),
                    state_desired_demux->get_input_port());
    builder.ExportOutput(state_desired_demux->get_output_port(1),
                         "velocity_commanded");
  } else {
    inverse_dynamics = builder.template AddNamedSystem<InverseDynamics>(
        "gravity_compensation", controller_plant,
        InverseDynamics<T>::InverseDynamicsMode::kGravityCompensation);
  }
  builder.ConnectInput("state",
                       inverse_dynamics->GetInputPort("estimated_state"));

  // Use the actuator indices to lookup limits parsed into the plant. Some
  // limits might be infinite, in which case the saturation won't do anything.
  auto& joint_actuator_indices = controller_plant->GetJointActuatorIndices();
  DRAKE_THROW_UNLESS(std::ssize(joint_actuator_indices) == num_positions);
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXd torque_limits =
      Eigen::VectorXd::Constant(num_positions, kNaN);
  for (int k = 0; k < num_positions; ++k) {
    const auto& actuator =
        controller_plant->get_joint_actuator(joint_actuator_indices[k]);
    torque_limits[k] = actuator.effort_limit();
  }
  auto torque_limiter = builder.template AddNamedSystem<systems::Saturation>(
      "torque_limiter", -torque_limits, torque_limits);

  // When torque control is enabled, declare the `torque` input port and add it
  // to the inverse dynamics output. Otherwise, use the inverse dynamics output
  // by itself.
  const systems::OutputPort<T>* raw_torque_commanded_output = nullptr;
  if (torque_enabled(control_mode)) {
    auto adder = builder.template AddNamedSystem<Adder>("+", 2, num_positions);
    builder.Connect(inverse_dynamics->GetOutputPort("generalized_force"),
                    adder->get_input_port(0));
    builder.ExportInput(adder->get_input_port(1), "torque");
    raw_torque_commanded_output = &adder->get_output_port();
  } else {
    raw_torque_commanded_output =
        &inverse_dynamics->GetOutputPort("generalized_force");
  }
  builder.Connect(*raw_torque_commanded_output,
                  torque_limiter->get_input_port());

  // Add B⁻¹ to the diagram.
  auto Binv = builder.template AddNamedSystem<systems::SparseMatrixGain>(
      "B⁻¹", controller_plant->MakeActuationMatrixPseudoinverse());
  builder.Connect(torque_limiter->get_output_port(), Binv->get_input_port());

  // Declare the various output ports.
  builder.ExportOutput(Binv->get_output_port(), "actuation");
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
  builder.ExportOutput(torque_limiter->get_output_port(), "torque_commanded");
  // TODO(amcastro-tri): is this what we want to send as the "measured
  // torque"? why coming from the controllers instead of from the plant?
  builder.ExportOutput(torque_limiter->get_output_port(), "torque_measured");
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
    const ModelInstanceIndex iiwa_instance, const IiwaDriver& driver_config,
    const MultibodyPlant<double>& controller_plant) {
  const std::string name = plant.GetModelInstanceName(iiwa_instance);
  auto system = builder->AddNamedSystem<SimIiwaDriver<double>>(
      name, driver_config, &controller_plant);
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
