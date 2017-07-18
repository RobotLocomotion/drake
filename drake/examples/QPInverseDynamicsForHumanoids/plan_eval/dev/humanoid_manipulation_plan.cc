#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/dev/humanoid_manipulation_plan.h"

#include "robotlocomotion/robot_plan_t.hpp"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
void HumanoidManipulationPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  // Knots are constant, the second time doesn't matter as long as it's larger.
  const std::vector<T> times = {robot_status.time(), robot_status.time() + 1};

  // Current com q and v.
  Vector4<double> xcom;
  xcom << robot_status.com().head<2>(), robot_status.comd().head<2>();
  // Set desired com q to current.
  MatrixX<double> com_d = robot_status.com().head<2>();
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
  for (const auto& name : tracked_body_names) {
    const RigidBody<T>* body = alias_groups.get_body(name);
    Isometry3<T> body_pose = robot_status.robot().CalcBodyPoseInWorldFrame(
        robot_status.cache(), *body);

    manipulation::PiecewiseCartesianTrajectory<T> body_traj =
        manipulation::PiecewiseCartesianTrajectory<
            T>::MakeCubicLinearWithEndLinearVelocity(times,
                                                     {body_pose, body_pose},
                                                     Vector3<T>::Zero(),
                                                     Vector3<T>::Zero());
    this->set_body_trajectory(body, body_traj);
  }
}

template <typename T>
void HumanoidManipulationPlan<T>::UpdateQpInputGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    QpInput* qp_input) const {
  // Generates CoM acceleration.
  Vector4<T> xcom;
  xcom << robot_status.com().head<2>(), robot_status.comd().head<2>();
  Vector2<T> comdd_d =
      zmp_planner_.ComputeOptimalCoMdd(robot_status.time(), xcom);

  // Zeros linear and angular momentum change.
  qp_input->mutable_desired_centroidal_momentum_dot().mutable_values().setZero();
  // Only sets the xy dimensions of the linear momentum change.
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .segment<2>(3) = robot_status.robot().getMass() * comdd_d;
}

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

  const manipulation::RobotStateLcmMessageTranslator translator(
      robot_status.robot());
  for (size_t f = 0; f < msg.plan.size(); ++f) {
    const bot_core::robot_state_t& keyframe = msg.plan[f];
    translator.DecodeMessageKinematics(keyframe, q, v);
    const double time = static_cast<double>(msg.utime) / 1e6;

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
    zmp_planner_.Plan(zmp_poly, x_com0, zmp_height_);
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
          manipulation::PiecewiseCartesianTrajectory<
              T>::MakeCubicLinearWithEndLinearVelocity(times, body_knots,
                                                       Vector3<T>::Zero(),
                                                       Vector3<T>::Zero());

      this->set_body_trajectory(body, body_traj);
    }
  }
}

template <typename T>
GenericPlan<T>* HumanoidManipulationPlan<T>::CloneGenericPlanDerived() const {
  return new HumanoidManipulationPlan<T>(*this);
}

template class HumanoidManipulationPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
