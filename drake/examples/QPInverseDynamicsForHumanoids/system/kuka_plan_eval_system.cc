#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_plan_eval_system.h"

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller_common.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"

#include "drake/common/trajectories/piecewise_quaternion.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

struct KukaPlan {
  VectorSetpoint<double> joint_PDff;

  PiecewisePolynomial<double> q_n_traj;
  PiecewisePolynomial<double> qd_n_traj;
  PiecewisePolynomial<double> qdd_n_traj;

  CartesianSetpoint<double> flange_PDff;
  PiecewisePolynomial<double> flange_pos_n_traj;
  PiecewiseQuaternionSlerp<double> flange_quat_n_traj;
};

void KukaPlanEvalSystem::SetDesiredTrajectory(const PiecewisePolynomialTrajectory& traj, systems::State<double>* state) {
  DRAKE_DEMAND(robot_.get_num_velocities() == traj.rows());

  KukaPlan& plan = get_mutable_plan<KukaPlan>(state);
  plan.q_n_traj = traj.get_piecewise_polynomial();
  plan.qd_n_traj = plan.q_n_traj.derivative();
  plan.qdd_n_traj = plan.qd_n_traj.derivative();

  plan.joint_PDff = VectorSetpoint<double>(robot_.get_num_velocities());
  paramset_.LookupDesiredDofMotionGains(&(plan.joint_PDff.mutable_Kp()),
                                        &(plan.joint_PDff.mutable_Kd()));

  // Setup cartesian
  const RigidBody<double>* flange = alias_groups_.get_body_group("flange").front();
  VectorX<double> q;
  std::vector<double> times = plan.q_n_traj.getSegmentTimes();
  std::vector<MatrixX<double>> flange_pos(times.size());
  eigen_aligned_std_vector<Matrix3<double>> flange_rot(times.size());
  for (size_t i = 0; i < times.size(); ++i) {
    q = plan.q_n_traj.value(times[i]);
    KinematicsCache<double> cache = robot_.doKinematics(q);
    Isometry3<double> flange_pose = robot_.CalcBodyPoseInWorldFrame(cache, *flange);
    flange_pos[i] = flange_pose.translation();
    flange_rot[i] = flange_pose.linear();
  }
  plan.flange_pos_n_traj = PiecewisePolynomial<double>::FirstOrderHold(times, flange_pos);
  plan.flange_quat_n_traj = PiecewiseQuaternionSlerp<double>(times, flange_rot);
  paramset_.LookupDesiredBodyMotionGains(*flange,
                                         &(plan.flange_PDff.mutable_Kp()),
                                         &(plan.flange_PDff.mutable_Kd()));
}

void KukaPlanEvalSystem::DoCalcOutput(const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {

  // Gets the plan from abstract state.
  const KukaPlan& plan = context.get_abstract_state<KukaPlan>(abstract_state_plan_index_);

  // Gets the robot state from input.
  const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
      context, input_port_index_humanoid_status_);

  // Output:
  QpInput& qp_input = output->GetMutableData(output_port_index_qp_input_)
                        ->GetMutableValue<QpInput>();

  qp_input = paramset_.MakeQpInput({}, {"flange"}, alias_groups_);

  double time = context.get_time();
  // TODO(siyuan): this copying won't be necessary once this untire block is moved to DoCalcUnrestrictedUpdate.
  VectorSetpoint<double> servo = plan.joint_PDff;
  servo.mutable_desired_position() = plan.q_n_traj.value(time);
  servo.mutable_desired_velocity() = plan.qd_n_traj.value(time);
  servo.mutable_desired_acceleration() = plan.qdd_n_traj.value(time);

  // Update desired accelerations.
  qp_input.mutable_desired_dof_motions().mutable_values() =
      servo.ComputeTargetAcceleration(robot_status->position(), robot_status->velocity());

  const RigidBody<double>* flange = alias_groups_.get_body_group("flange").front();
  CartesianSetpoint<double> flange_servo = plan.flange_PDff;
  // Measured
  Isometry3<double> flange_pose = robot_.CalcBodyPoseInWorldFrame(robot_status->cache(), *flange);
  Vector6<double> flange_vel = robot_.CalcBodySpatialVelocityInWorldFrame(robot_status->cache(), *flange);
  // Desired
  flange_servo.mutable_desired_pose().translation() = plan.flange_pos_n_traj.value(time);
  flange_servo.mutable_desired_velocity().tail<3>() = plan.flange_pos_n_traj.derivative().value(time);
  flange_servo.mutable_desired_pose().linear() = Matrix3<double>(plan.flange_quat_n_traj.orientation(time));
  flange_servo.mutable_desired_velocity().head<3>() = plan.flange_quat_n_traj.angular_velocity(time);

  qp_input.mutable_desired_body_motions().at(flange->get_name()).mutable_values() =
      flange_servo.ComputeTargetAcceleration(flange_pose, flange_vel);
}

void KukaPlanEvalSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // TODO(siyuan): move stuff in DoCalcOutput to here once we have CalcUnrestrictedUpdate in diagram.
}

std::unique_ptr<systems::AbstractState> KukaPlanEvalSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals(2);
  abstract_vals[abstract_state_plan_index_] = std::move(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<KukaPlan>(KukaPlan())));
  abstract_vals[abstract_state_qp_input_index_] = std::move(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<QpInput>(QpInput(GetDofNames(robot_)))));
  return std::make_unique<systems::AbstractState>(std::move(abstract_vals));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
