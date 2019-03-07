#include "drake/examples/humanoid_controller/dev/humanoid_manipulation_plan.h"

#include <string>
#include <unordered_map>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/systems/controllers/qp_inverse_dynamics/lcm_utils.h"
#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

using systems::controllers::plan_eval::ContactState;
using systems::controllers::qp_inverse_dynamics::ParamSet;
using systems::controllers::qp_inverse_dynamics::QpInput;
using systems::controllers::qp_inverse_dynamics::RobotKinematicState;
using trajectories::PiecewisePolynomial;
using trajectories::PiecewiseQuaternionSlerp;

template <typename T>
void HumanoidManipulationPlan<T>::InitializeGenericPlanDerived(
    const RobotKinematicState<T>& robot_status, const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups) {
  unused(paramset);

  // Knots are constant, the second time doesn't matter as long as it's larger.
  const std::vector<T> times = {robot_status.get_time(),
                                robot_status.get_time() + 1};

  // Current com q and v.
  Vector4<T> xcom;
  xcom << robot_status.get_com().template head<2>(),
      robot_status.get_com_velocity().template head<2>();
  // Set desired com q to current.
  MatrixX<T> com_d = robot_status.get_com().template head<2>();
  PiecewisePolynomial<T> zmp_d =
      PiecewisePolynomial<T>::ZeroOrderHold(times, {com_d, com_d});
  // Makes a zmp planner that stays still.
  zmp_planner_.Plan(zmp_d, xcom, zmp_height_);

  // Assumes double support with both feet.
  ContactState double_support;
  double_support.insert(alias_groups.get_body("left_foot"));
  double_support.insert(alias_groups.get_body("right_foot"));
  this->UpdateContactState(double_support);

  // Sets body tracking trajectories for pelvis and torso.
  const std::vector<std::string> tracked_body_names = {"pelvis", "torso"};
  MatrixX<T> position;
  for (const auto& name : tracked_body_names) {
    const RigidBody<T>* body = alias_groups.get_body(name);
    Isometry3<T> body_pose = robot_status.get_robot().CalcBodyPoseInWorldFrame(
        robot_status.get_cache(), *body);
    position = body_pose.translation();
    PiecewisePolynomial<T> pos_traj =
        PiecewisePolynomial<T>::ZeroOrderHold(times, {position, position});
    PiecewiseQuaternionSlerp<T> rot_traj(
        times, {body_pose.linear(), body_pose.linear()});

    manipulation::PiecewiseCartesianTrajectory<T> body_traj(pos_traj, rot_traj);
    this->set_body_trajectory(body, body_traj);
  }
}

template <typename T>
void HumanoidManipulationPlan<T>::UpdateQpInputGenericPlanDerived(
    const RobotKinematicState<T>& robot_status, const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups, QpInput* qp_input) const {
  unused(paramset, alias_groups);

  // Generates CoM acceleration.
  Vector4<T> xcom;
  xcom << robot_status.get_com().template head<2>(),
      robot_status.get_com_velocity().template head<2>();
  Vector2<T> comdd_d =
      zmp_planner_.ComputeOptimalCoMdd(robot_status.get_time(), xcom);

  // Zeros linear and angular momentum change.
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .setZero();
  // Only sets the xy dimensions of the linear momentum change.
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .segment<2>(3) = robot_status.get_robot().getMass() * comdd_d;
}

template <typename T>
void HumanoidManipulationPlan<T>::HandlePlanGenericPlanDerived(
    const RobotKinematicState<T>& robot_status, const ParamSet& paramset,
    const RigidBodyTreeAliasGroups<T>& alias_groups,
    const AbstractValue& plan) {
  unused(paramset);

  const auto& msg = plan.get_value<robotlocomotion::robot_plan_t>();

  if (msg.utime == last_handle_plan_time_) return;

  // Saves the time stamp.
  last_handle_plan_time_ = msg.utime;

  // knots for setting up the splines.
  int length = static_cast<int>(msg.plan.size());
  if (length < 1) {
    drake::log()->warn(
        "HumanoidManipulationPlan::HandlePlanGenericPlanDerived: "
        "received plan has less than 1 knots.");
    return;
  }

  const RigidBodyTree<T>& robot = robot_status.get_robot();
  KinematicsCache<T> cache = robot.CreateKinematicsCache();

  const double time_now = robot_status.get_time();
  VectorX<T> q = this->get_dof_trajectory().get_position(time_now);
  VectorX<T> v = VectorX<T>::Zero(robot.get_num_velocities());

  // Set the first knot points to the current desired values.
  std::vector<T> times(1, time_now);
  std::vector<MatrixX<T>> dof_knots(1, q);
  std::unordered_map<const RigidBody<T>*, std::vector<Isometry3<T>>> body_knots;
  std::vector<const RigidBody<T>*> tracked_bodies = {
      alias_groups.get_body("pelvis"), alias_groups.get_body("torso")};
  for (const RigidBody<T>* body : tracked_bodies) {
    body_knots[body] = std::vector<Isometry3<T>>(
        1, this->get_body_trajectory(body).get_pose(time_now));
  }
  std::vector<T> com_times(1, time_now);
  std::vector<MatrixX<T>> com_knots(1, zmp_planner_.get_nominal_com(time_now));

  const manipulation::RobotStateLcmMessageTranslator translator(
      robot_status.get_robot());

  for (const bot_core::robot_state_t& keyframe : msg.plan) {
    translator.DecodeMessageKinematics(keyframe, q, v);
    const double time = static_cast<double>(keyframe.utime) / 1e6;

    cache.initialize(q);
    robot.doKinematics(cache, false);

    times.push_back(time_now + time);
    com_times.push_back(time_now + time);
    dof_knots.push_back(q);

    for (auto& body_knots_pair : body_knots) {
      const RigidBody<T>* body = body_knots_pair.first;
      std::vector<Isometry3<T>>& knots = body_knots_pair.second;
      knots.push_back(robot.CalcBodyPoseInWorldFrame(cache, *body));
    }

    // Computes com.
    com_knots.push_back(robot.centerOfMass(cache).template head<2>());
  }

  // Generates the zmp trajectory.
  {
    // CoM has 1 more knot point to ensure the trajectory ends at zero velocity.
    // TODO(siyuan): have the zmp planner deal with this.
    // Repeats the last desired CoM position to ensure CoM trajector ends in
    // zero velocity.
    com_times.push_back(com_times.back() + 0.1);
    com_knots.push_back(com_knots.back());
    PiecewisePolynomial<T> zmp_poly =
        PiecewisePolynomial<T>::Pchip(com_times, com_knots, true);
    Vector4<T> x_com0;
    x_com0 << robot_status.get_com().template head<2>(),
        robot_status.get_com_velocity().template head<2>();
    zmp_planner_.Plan(zmp_poly, x_com0, zmp_height_);
  }

  // Generates dof trajectories.
  {
    MatrixX<T> zeros = MatrixX<T>::Zero(robot.get_num_positions(), 1);
    this->set_dof_trajectory(manipulation::PiecewiseCubicTrajectory<T>(
        PiecewisePolynomial<T>::Cubic(times, dof_knots, zeros, zeros)));
  }

  // Generates body trajectories.
  {
    for (const auto& body_knots_pair : body_knots) {
      const RigidBody<T>* body = body_knots_pair.first;
      const std::vector<Isometry3<T>>& knots = body_knots_pair.second;

      manipulation::PiecewiseCartesianTrajectory<T> body_traj =
          manipulation::PiecewiseCartesianTrajectory<
              T>::MakeCubicLinearWithEndLinearVelocity(times, knots,
                                                       Vector3<T>::Zero(),
                                                       Vector3<T>::Zero());

      this->set_body_trajectory(body, body_traj);
    }
  }
}

template <typename T>
bool HumanoidManipulationPlan<T>::IsRigidBodyTreeCompatible(
    const RigidBodyTree<T>& robot) const {
  if (robot.get_num_bodies() < 2) return false;

  const RigidBody<T>& root_body = robot.get_body(1);
  const DrakeJoint& root_joint = root_body.getJoint();
  if (!root_joint.is_floating()) return false;

  if (root_joint.get_num_positions() != root_joint.get_num_velocities())
    return false;

  if (robot.get_num_positions() != robot.get_num_velocities()) return false;

  return true;
}

template <typename T>
bool HumanoidManipulationPlan<T>::IsRigidBodyTreeAliasGroupsCompatible(
    const RigidBodyTreeAliasGroups<T>& alias_groups) const {
  if (VerifyRigidBodyTreeAliasGroups(alias_groups, "pelvis") &&
      VerifyRigidBodyTreeAliasGroups(alias_groups, "torso") &&
      VerifyRigidBodyTreeAliasGroups(alias_groups, "left_foot") &&
      VerifyRigidBodyTreeAliasGroups(alias_groups, "right_foot")) {
    return true;
  } else {
    return false;
  }
}

template class HumanoidManipulationPlan<double>;

}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
