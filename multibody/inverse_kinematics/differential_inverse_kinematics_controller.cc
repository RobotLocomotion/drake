#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/planning/dof_mask.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/selector.h"
#include "drake/systems/primitives/wrap_to_system.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::VectorXd;
using planning::DofMask;
using systems::Context;
using systems::DiagramBuilder;
using systems::DiscreteTimeIntegrator;
using systems::Selector;
using systems::SelectorParams;
using systems::State;
using systems::WrapToSystem;

using Base = systems::Diagram<double>;

// Map active and passive degrees of freedom from input to output for the
// position mixer.
SelectorParams set_active_passive_dof_selection(const DofMask& active_dof) {
  SelectorParams result;
  SelectorParams::InputPortParams active;
  SelectorParams::InputPortParams passive;
  passive.size = 2 * active_dof.size();
  SelectorParams::OutputPortParams output;
  int ones{0};
  for (int k = 0; k < active_dof.size(); ++k) {
    if (active_dof[k] == true) {
      output.selections.push_back(SelectorParams::OutputSelection{0, ones});
      ++ones;
    } else {
      output.selections.push_back(SelectorParams::OutputSelection{1, k});
    }
  }
  active.size = ones;
  result.inputs = {active, passive};
  result.outputs = {output};
  return result;
}

}  // namespace

DifferentialInverseKinematicsController::
    DifferentialInverseKinematicsController(
        std::shared_ptr<DifferentialInverseKinematicsSystem>
            differential_inverse_kinematics,
        const std::vector<int>& planar_rotation_dof_indices) {
  DiagramBuilder<double> builder;

  differential_inverse_kinematics_ =
      builder.template AddSystem<DifferentialInverseKinematicsSystem>(
          std::move(differential_inverse_kinematics));

  const DofMask& active_dof = differential_inverse_kinematics_->active_dof();
  const int active_dof_count = active_dof.count();
  discrete_time_integrator_ =
      builder.template AddSystem<DiscreteTimeIntegrator<double>>(
          active_dof_count, differential_inverse_kinematics_->time_step());

  // Initialize the position mixer to get active dof values from output of the
  // integrator and passive dof values from the output of the estimated state.
  auto* position_mixer = builder.template AddSystem<Selector<double>>(
      set_active_passive_dof_selection(active_dof));

  auto* wrap_to_system =
      builder.template AddSystem<WrapToSystem<double>>(active_dof_count);
  for (const auto& index : planar_rotation_dof_indices) {
    // TODO(rpoyner-tri): not reached by unit tests.
    wrap_to_system->set_interval(index, -M_PI, M_PI);
  }

  builder.ExportInput(position_mixer->get_input_port(1), "estimated_state");
  builder.ExportInput(differential_inverse_kinematics_
                          ->get_input_port_desired_cartesian_poses(),
                      "desired_poses");
  builder.ExportInput(
      differential_inverse_kinematics_->get_input_port_nominal_posture(),
      "nominal_posture");

  builder.Connect(
      differential_inverse_kinematics_->get_output_port_commanded_velocity(),
      discrete_time_integrator_->get_input_port());

  builder.Connect(discrete_time_integrator_->get_output_port(),
                  position_mixer->get_input_port(0));

  builder.Connect(position_mixer->get_output_port(),
                  differential_inverse_kinematics_->get_input_port_position());
  builder.Connect(discrete_time_integrator_->get_output_port(),
                  wrap_to_system->get_input_port());

  builder.ExportOutput(wrap_to_system->get_output_port(), "commanded_position");
  builder.ExportOutput(
      differential_inverse_kinematics_->get_output_port_commanded_velocity(),
      "commanded_velocity");

  builder.BuildInto(this);
}

DifferentialInverseKinematicsController::
    ~DifferentialInverseKinematicsController() = default;

void DifferentialInverseKinematicsController::set_initial_position(
    Context<double>* context, const Eigen::Ref<const VectorXd>& value) const {
  Context<double>& discrete_time_integrator_context =
      GetMutableSubsystemContext(*discrete_time_integrator_, context);
  const DofMask& active_dof = differential_inverse_kinematics_->active_dof();
  discrete_time_integrator_->set_integral_value(
      &discrete_time_integrator_context, active_dof.GetFromArray(value));
}

// Set default state of the discrete time integrator and the zero order hold to
// NaN because we want this diagram to fail fast if the user did not explicitly
// initialize the values.
void DifferentialInverseKinematicsController::SetDefaultState(
    const Context<double>& context, State<double>* state) const {
  Base::SetDefaultState(context, state);
  set_state_to_nan(context, state);
}

void DifferentialInverseKinematicsController::SetRandomState(
    const Context<double>& context, State<double>* state,
    RandomGenerator* generator) const {
  Base::SetRandomState(context, state, generator);
  // This is a state that effectively depends on connected ports; we should not
  // randomly initialize this, and instead still require explicit initialization
  // be done.
  set_state_to_nan(context, state);
}

void DifferentialInverseKinematicsController::set_state_to_nan(
    const Context<double>&, State<double>* state) const {
  VectorXd integrator_value(
      GetMutableSubsystemState(*discrete_time_integrator_, state)
          .get_discrete_state()
          .size());
  integrator_value.fill(std::numeric_limits<double>::quiet_NaN());
  GetMutableSubsystemState(*discrete_time_integrator_, state)
      .get_mutable_discrete_state(0)
      .SetFromVector(integrator_value);
}

}  // namespace multibody
}  // namespace drake
