#pragma once

#include <string>

#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/systems/controllers/plan_eval/generic_plan.h"
#include "drake/systems/controllers/zmp_planner.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

// TODO(siyuan): make quaternion floating joint work.
/**
 * A baseline manipulation plan interpretor for a humanoid robot. The plan
 * essentially consists of a sequence of time and generalized positions,
 * which are used to generate splines, which are then used as desired
 * trajectories to populate QpInput for the QPController.
 *
 * @see HandlePlanGenericPlanDerived for more details about the behavior.
 */
template <typename T>
class HumanoidManipulationPlan
    : public systems::controllers::plan_eval::GenericPlan<T> {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HumanoidManipulationPlan)

 public:
  HumanoidManipulationPlan() {}

  /**
   * Returns the center of mass height used in the ZMP planner.
   * @see ZMPPlanner::Plan for more details.
   */
  double get_zmp_height() const { return zmp_height_; }

  /**
   * Sets the center of mass height used in the ZMP planner to @p z.
   * @see ZMPPlanner::Plan for more details.
   */
  void set_zmp_height(double z) { zmp_height_ = z; }

  /**
   * Returns true if @p robot has at least 2 rigid bodies; the first rigid body
   * has a RPY parameterized floating joint, and it has the same number of
   * generalized positions and velocities.
   */
  bool IsRigidBodyTreeCompatible(const RigidBodyTree<T>& robot) const override;

  /**
   * Returns true if @p alias_groups has singleton body groups: "pelvis",
   * "torso", "left_foot" and "right_foot".
   */
  bool IsRigidBodyTreeAliasGroupsCompatible(
      const RigidBodyTreeAliasGroups<T>& alias_groups) const override;

  // TODO(siyuan): override IsParamSetCompatible as well.

 protected:
  /**
   * Assuming both feet are in contact, initializes a plan that holds the
   * current configuration in @p robot_status.
   */
  void InitializeGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups) override;

  /**
   * This function generates various trajectories from the given plan in
   * @p plan. Let `T_plan` be the timing information and `Q_plan` be the
   * generalized positions contained in @p plan, and `t` and `q` be the current
   * time and estimated generalized position represented by @p robot_status,
   * we define `Ts = {t, T_plan + t}` and `Qs = {q, Q_plan}`.
   * Given timing `Ts` and knot points for the generalized positions `Qs`, the
   * following trajectories are generated:
   * <pre>
   * 1. Cubic splines for all degrees of freedom. The splines are generated from
   *    `Ts` and `Qs` assuming zero end point velocities and continuous
   * position,
   *    velocity and acceleration at all intermediate knot points.
   * 2. Cartesian trajectories are made for the pelvis and torso link. The knot
   *    poses are computed by forward kinematics at each `Qs`. The linear part
   * of
   *    the Cartesian trajectory is constructed similarly as above, and the
   *    rotation part is based on Slerp (linear interpolation between
   * quaternion).
   *    Timing is given by `Ts`.
   * 3. A desired ZMP trajectory is generated using Pchip from the center of
   * mass
   *    XY positions computed by forward kinematics at each `Qs` and timing
   * given
   *    by `Ts`. A nominal center of mass trajectory and its associated optimal
   *    linear policy is then planned using systems::ZMPPlanner.
   * </pre>
   * Note that these trajectories are arbitrarily designed, without considering
   * position, velocity or acceleration consistency. The QpController will trade
   * off tracking errors based on weights specified in the parameters.
   *
   * This function also assumes that both feet are in contact with the ground,
   * and every configuration in `Q_plan` is statically stable given the support
   * region defined by the current foot poses. `T_plan` has to be strictly
   * increasing, and `T_plan[0]` is bigger than 0. The current implementation
   * assumes that the message is encoded with a RPY parameterized floating base
   * model as opposed to a quaternion floating joint.
   *
   * This function returns without changing the current trajectories if the
   * time stamp in @p plan has not changed from when the current trajectories
   * were last generated or the number of knots in @p plan is smaller than 1.
   *
   * Aborts if assumptions about `T_plan` is not valid.
   *
   * @throws std::exception if @p plan is not of type
   * robotlocomotion::robot_plan_t
   */
  void HandlePlanGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      const AbstractValue& plan) override;

  /**
   * Updates the X and Y dimension of the desired linear momentum change in
   * @p qp_input based on the center of mass acceleration computed using the
   * ZMP planner's policy. Sets the other dimensions of desired centroidal
   * momentum change to zero.
   */
  void UpdateQpInputGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&
          robot_status,
      const systems::controllers::qp_inverse_dynamics::ParamSet& paramset,
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      systems::controllers::qp_inverse_dynamics::QpInput* qp_input)
      const override;

 private:
  // Returns true if @p alias_groups has a singleton body group whose name is
  // @p body_alias_name.
  bool VerifyRigidBodyTreeAliasGroups(
      const RigidBodyTreeAliasGroups<T>& alias_groups,
      const std::string& body_alias_name) const {
    if (alias_groups.has_body_group(body_alias_name) &&
        alias_groups.get_body_group(body_alias_name).size() == 1) {
      return true;
    } else {
      return false;
    }
  }

  systems::controllers::plan_eval::GenericPlan<T>* CloneGenericPlanDerived()
      const override {
    return new HumanoidManipulationPlan<T>(*this);
  }

  void ModifyPlanGenericPlanDerived(
      const systems::controllers::qp_inverse_dynamics::RobotKinematicState<T>&,
      const systems::controllers::qp_inverse_dynamics::ParamSet&,
      const RigidBodyTreeAliasGroups<T>&) override {}

  systems::controllers::ZMPPlanner zmp_planner_;
  double zmp_height_{1.0};
  int64_t last_handle_plan_time_{-1};
};

}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
