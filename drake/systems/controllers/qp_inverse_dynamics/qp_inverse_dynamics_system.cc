#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_system.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/lcmt_inverse_dynamics_debug_info.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {
namespace {

template <typename ValueType>
ValueType& get_mutable_value(systems::State<double>* state, int index) {
  DRAKE_DEMAND(state);
  return state->get_mutable_abstract_state()
      .get_mutable_value(index)
      .GetMutableValue<ValueType>();
}

}  // namespace

QpInverseDynamicsSystem::QpInverseDynamicsSystem(
    const RigidBodyTree<double>* robot, double dt)
    : robot_(*robot), control_dt_(dt) {
  input_port_index_kinematic_state_ = DeclareAbstractInputPort().get_index();
  input_port_index_qp_input_ = DeclareAbstractInputPort().get_index();
  output_port_index_qp_output_ =
      DeclareAbstractOutputPort(QpOutput(GetDofNames(robot_)),
                                &QpInverseDynamicsSystem::CopyOutQpOutput)
          .get_index();
  output_port_index_debug_info_ =
      DeclareAbstractOutputPort(lcmt_inverse_dynamics_debug_info(),
                                &QpInverseDynamicsSystem::CopyOutDebugInfo)
          .get_index();
  DeclarePeriodicUnrestrictedUpdate(control_dt_, 0);

  abs_state_index_qp_output_ = DeclareAbstractState(
      systems::AbstractValue::Make<QpOutput>(QpOutput(GetDofNames(robot_))));
  abs_state_index_debug_info_ = DeclareAbstractState(
      systems::AbstractValue::Make<lcmt_inverse_dynamics_debug_info>(
          lcmt_inverse_dynamics_debug_info()));
}

void QpInverseDynamicsSystem::CopyOutQpOutput(
    const systems::Context<double>& context,
    QpOutput* output) const {
  QpOutput& qp_output = *output;
  qp_output = context.get_abstract_state<QpOutput>(abs_state_index_qp_output_);
}

void QpInverseDynamicsSystem::CopyOutDebugInfo(
    const systems::Context<double>& context,
    lcmt_inverse_dynamics_debug_info* output) const {
  lcmt_inverse_dynamics_debug_info& debug = *output;
  debug = context.get_abstract_state<lcmt_inverse_dynamics_debug_info>(
      abs_state_index_debug_info_);
}

void QpInverseDynamicsSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Inputs:
  const RobotKinematicState<double>* rs =
      EvalInputValue<RobotKinematicState<double>>(
          context, input_port_index_kinematic_state_);

  const QpInput* qp_input =
      EvalInputValue<QpInput>(context, input_port_index_qp_input_);

  // Calls the controller.
  QpOutput& qp_output =
      get_mutable_value<QpOutput>(state, abs_state_index_qp_output_);

  if (qp_controller_.Control(*rs, *qp_input, &qp_output) < 0) {
    std::stringstream err;
    err << rs->get_cache().getQ().transpose() << "\n";
    err << rs->get_cache().getV().transpose() << "\n";
    err << *qp_input << std::endl;
    throw std::runtime_error("QpInverseDynamicsSystem: QP cannot solve\n" +
                             err.str());
  }

  // Generates debugging info.
  lcmt_inverse_dynamics_debug_info& debug =
      get_mutable_value<lcmt_inverse_dynamics_debug_info>(
          state, abs_state_index_debug_info_);

  debug.timestamp = context.get_time() * 1e6;
  debug.num_dof = robot_.get_num_velocities();
  debug.dof_names = GetDofNames(robot_);
  debug.desired_vd.resize(debug.num_dof);
  debug.solved_vd.resize(debug.num_dof);
  debug.solved_torque.resize(debug.num_dof);
  for (int i = 0; i < debug.num_dof; ++i) {
    debug.desired_vd[i] = qp_input->desired_dof_motions().value(i);
    debug.solved_vd[i] = qp_output.vd()[i];
    debug.solved_torque[i] = qp_output.dof_torques()[i];
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
