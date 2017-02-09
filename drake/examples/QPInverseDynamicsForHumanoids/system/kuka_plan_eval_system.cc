#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_plan_eval_system.h"

#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"

#include "drake/common/trajectories/piecewise_quaternion.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// An example plan that follows a desired trajetory.
struct KukaPlan {
  VectorSetpoint<double> joint_PDff;

  PiecewisePolynomial<double> q_n_traj;
  PiecewisePolynomial<double> qd_n_traj;
  PiecewisePolynomial<double> qdd_n_traj;
};

KukaPlanEvalSystem::KukaPlanEvalSystem(
    const RigidBodyTree<double>& robot,
    const std::string& alias_groups_file_name,
    const std::string& param_file_name, double dt)
    : DiscreteTimePlanEvalSystem(robot, alias_groups_file_name, param_file_name,
                                 dt) {
  set_name("kuka_plan_eval");
}

void KukaPlanEvalSystem::SetDesiredTrajectory(
    const PiecewisePolynomialTrajectory& traj, systems::State<double>* state) {
  DRAKE_DEMAND(get_robot().get_num_velocities() == traj.rows());

  KukaPlan& plan = get_mutable_plan<KukaPlan>(state);
  plan.q_n_traj = traj.get_piecewise_polynomial();
  plan.qd_n_traj = plan.q_n_traj.derivative();
  plan.qdd_n_traj = plan.qd_n_traj.derivative();

  plan.joint_PDff = VectorSetpoint<double>(get_robot().get_num_velocities());
  get_paramset().LookupDesiredDofMotionGains(&(plan.joint_PDff.mutable_Kp()),
                                             &(plan.joint_PDff.mutable_Kd()));
}

void KukaPlanEvalSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Gets the plan from abstract state.
  KukaPlan& plan = get_mutable_plan<KukaPlan>(state);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, get_input_port_index_humanoid_status());

  QpInput& qp_input = get_mutable_qp_input(state);
  qp_input = get_paramset().MakeQpInput({}, /* contacts */
                                        {}, /* tracked bodies */
                                        get_alias_groups());

  double time = context.get_time();
  plan.joint_PDff.mutable_desired_position() = plan.q_n_traj.value(time);
  plan.joint_PDff.mutable_desired_velocity() = plan.qd_n_traj.value(time);
  plan.joint_PDff.mutable_desired_acceleration() = plan.qdd_n_traj.value(time);

  // Set desired velocity and acceleration to zero if we run past the end of
  // the trajectory.
  if (time > plan.q_n_traj.getEndTime()) {
    plan.joint_PDff.mutable_desired_velocity().setZero();
    plan.joint_PDff.mutable_desired_acceleration().setZero();
  }

  // Update desired accelerations.
  qp_input.mutable_desired_dof_motions().mutable_values() =
      plan.joint_PDff.ComputeTargetAcceleration(robot_status->position(),
                                                robot_status->velocity());
}

std::unique_ptr<systems::AbstractState>
KukaPlanEvalSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  abstract_vals[get_abstract_state_index_plan()] =
      std::unique_ptr<systems::AbstractValue>(
          new systems::Value<KukaPlan>(KukaPlan()));
  abstract_vals[get_abstract_state_index_qp_input()] =
      std::unique_ptr<systems::AbstractValue>(new systems::Value<QpInput>(
          get_paramset().MakeQpInput({}, /* contacts */
                                     {}, /* tracked bodies */
                                     get_alias_groups())));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
