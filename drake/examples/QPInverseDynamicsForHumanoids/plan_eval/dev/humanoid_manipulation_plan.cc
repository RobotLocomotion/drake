#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/dev/humanoid_manipulation_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
HumanoidPlan<T>* HumanoidManipulationPlan<T>::CloneHumanoidPlanDerived() const {
  HumanoidManipulationPlan<T>* clone = new HumanoidManipulationPlan<T>();
  return clone;
}

template <typename T>
void HumanoidManipulationPlan<T>::InitializeHumanoidPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

template <typename T>
void HumanoidManipulationPlan<T>::ModifyPlanGenericPlanDerived(
    const HumanoidStatus& robot_stauts, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {}

template <typename T>
void HumanoidManipulationPlan<T>::HandlePlanMessageGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    const void* message_bytes, int message_length) {
  robotlocomotion::robot_plan_t msg;
  int consumed = msg.decode(message_bytes, 0, message_length);
  DRAKE_DEMAND(consumed == message_length);

  const RigidBodyTree<T>& robot = robot_status.robot();
  KinematicsCache<T> cache = robot.CreateKinematicsCache();

  VectorX<T> q(robot.get_num_positions());
  VectorX<T> v(robot.get_num_velocities());

  // knots for setting up the splines.
  int length = static_cast<int>(msg.plan.size());
  std::vector<T> times(length);
  std::vector<MatrixX<T>> dof_knots(length);
  std::vector<MatrixX<T>> com_knots(length);

  std::unordered_map<const RigidBody<T>*, std::vector<Isometry3<T>>> body_knots;
  body_knots[alias_groups.get_body("pelvis")] =
      std::vector<Isometry3<T>>(length);
  body_knots[alias_groups.get_body("torso")] =
      std::vector<Isometry3<T>>(length);

  for (size_t f = 0; f < msg.plan.size(); ++f) {
    const bot_core::robot_state_t& keyframe = msg.plan[f];
    translator_.DecodeMessageKinematics(keyframe, q, v);
    const double time = static_cast<double>(msg->utime) / 1e6;

    cache.initialize(q);
    robot.doKinematics(cache, false);

    times[f] = robot_status.time() + time;
    dof_knots[f] = q;
    dof_knots[f]
        .template block<6, 1>(0, 0)
        .setZero();  // don't interp the floating base in joint space.

    // TODO, list of tracked bodies. this should be contained in the msg.
    // but it's not now. so just assume we track pelvis and torso.
    for (auto& body_knots_pair : body_knots) {
      const RigidBody<T>* body = body_knots_pair.first;
      std::vector<Isometry3<T>>& body_knots = body_knots_pair.second;
      body_knots[f] = robot.CalcBodyPoseInWorldFrame(cache, *body);
    }

    // Computes com.
    com_knots[f] = robot.centerOfMass(cache).template head<2>();
    std::cout << com_knots[f].transpose() << std::endl;
  }

  // Generates the zmp trajectory.
  {
    PiecewisePolynomial<T> zmp_poly =
        PiecewisePolynomial<T>::Pchip(times, com_knots, true);
    Vector4<T> x_com0;
    x_com0 << robot_status.com().head<2>(), robot_status.comd().head<2>();
    // TODO: fix this
    const T height = 1;
    this->UpdateZmpPlan(zmp_poly, x_com0, height);
  }

  // Generates dof trajectories.
  {
    MatrixX<T> zeros = VectorX<T>::Zero(robot.get_num_positions());
    this->set_dof_trajectory(manipulation::PiecewiseCubicTrajectory<T>(
        PiecewisePolynomial<T>::Cubic(times, dof_knots, zeros, zeros)));
  }

  // Generates body trajectories.
  {
    for (const auto& body_knots_pair : body_knots) {
      const RigidBody<T>* body = body_knots_pair.first;
      const std::vector<Isometry3<T>>& body_knots = body_knots_pair.second;

      manipulation::PiecewiseCartesianTrajectory<T> body_traj =
          manipulation::PiecewiseCartesianTrajectory<T>::MakeCubicLinearWithEndLinearVelocity(
              times, body_knots, Vector3<T>::Zero(), Vector3<T>::Zero());

      this->set_body_trajectory(body, body_traj);
    }
  }
}

template class HumanoidManipulationPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
