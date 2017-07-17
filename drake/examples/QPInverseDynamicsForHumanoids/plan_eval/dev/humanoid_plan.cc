#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/dev/humanoid_plan.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
GenericPlan<T>* HumanoidPlan<T>::CloneGenericPlanDerived() const {
  HumanoidPlan<T>* clone = CloneHumanoidPlanDerived();
  clone->zmp_planner_ = this->zmp_planner_;

  return clone;
}

template <typename T>
void HumanoidPlan<T>::InitializeGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) {
  std::cout << "HPlan init.\n";

  // Knots are constant, the second time doesn't matter.
  const std::vector<T> times = {robot_status.time(), robot_status.time() + 1};

  // Makes a zmp planner that stays still.
  Vector4<double> xcom;
  xcom << robot_status.com().head<2>(), robot_status.comd().head<2>();
  MatrixX<double> com_d = robot_status.com().head<2>();
  PiecewisePolynomial<T> zmp_d =
      PiecewisePolynomial<T>::ZeroOrderHold(times, {com_d, com_d});
  this->UpdateZmpPlan(zmp_d, xcom, 1);

  // Sets contact state to double support always.
  ContactState double_support;
  double_support.insert(alias_groups.get_body("left_foot"));
  double_support.insert(alias_groups.get_body("right_foot"));
  this->UpdateContactState(double_support);

  // Sets body traj for pelvis and torso.
  const std::vector<std::string> tracked_body_names = {"pelvis", "torso"};
  for (const auto& name : tracked_body_names) {
    const RigidBody<T>* body = alias_groups.get_body(name);
    Isometry3<T> body_pose = robot_status.robot().CalcBodyPoseInWorldFrame(
        robot_status.cache(), *body);

    manipulation::PiecewiseCartesianTrajectory<T> body_traj =
      manipulation::PiecewiseCartesianTrajectory<T>::MakeCubicLinearWithEndLinearVelocity(
          times, {body_pose, body_pose}, Vector3<T>::Zero(), Vector3<T>::Zero());
    this->set_body_trajectory(body, body_traj);
  }

  // Calls derived classes' initialization.
  InitializeHumanoidPlanDerived(robot_status, paramset, alias_groups);
}

template <typename T>
void HumanoidPlan<T>::UpdateQpInputGenericPlanDerived(
    const HumanoidStatus& robot_status, const param_parsers::ParamSet& paramset,
    const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
    QpInput* qp_input) const {
  // Generates CoM acceleration.
  Vector4<T> xcom;
  xcom << robot_status.com().head<2>(), robot_status.comd().head<2>();
  Vector2<T> comdd_d = zmp_planner_.ComputeOptimalCoMdd(robot_status.time(), xcom);
  qp_input->mutable_desired_centroidal_momentum_dot()
      .mutable_values()
      .segment<2>(3) = robot_status.robot().getMass() * comdd_d;
}

template class HumanoidPlan<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
