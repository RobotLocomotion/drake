#pragma once

#include "drake/examples/planar_gripper/gripper_brick.h"
#include "drake/solvers/mathematical_program.h"

/// @file This file contains the utility function to add constraint in
/// gripper/brick motion planning.

namespace drake {
namespace examples {
namespace planar_gripper {
/**
 * Adds the friction cone as linear constraints on the contact force f_Cb_B (the
 * contact force applied on the brick contact point Cb, expressed in the brick
 * frame B). Notice that since the example is planar, we can write this friction
 * cone as linear constraints, as opposed to the general nonlinear constraint in
 * StaticFrictionConeConstraint.
 * @param gripper_brick The planar gripper system manipulating a brick.
 * @param finger The finger in contact with the brick face.
 * @param brick_face The contact facet on the brick.
 * @param f_Cb_B The contact force applied on the brick contact point Cb,
 * expressed in the brick frame B.
 * @param friction_cone_shrink_factor The factor to shrink the friction
 * coefficient mu in the planning.
 * @param prog The program to which the constraint is added.
 */
template <typename T>
void AddFrictionConeConstraint(
    const GripperBrickHelper<T>& gripper_brick, Finger finger,
    BrickFace brick_face,
    const Eigen::Ref<const Vector2<symbolic::Variable>>& f_Cb_B,
    double friction_cone_shrink_factor, solvers::MathematicalProgram* prog);

/**
 * Add the kinematic constraint that the finger tip (the sphere collision
 * geometry in the tip of the finger) is in contact with a shrunk region on the
 * brick face.
 * @param gripper_brick_system The gripper brick system on which the constraint
 * is imposed.
 * @param finger The finger in contact.
 * @param brick_face The face of the brick in contact.
 * @param prog The program to which the constraint is added.
 * @param q_vars The variable for the configuration.
 * @param plant_context The context containing the value of q_vars.
 * @param face_shrink_factor A factor to determine the shrunk region on each
 * face. If face_shrink_factor = 1, then the region includes the whole face,
 * 0 < face_shrink_factor < 1 corresponds to the scaled region of the face,
 * if face_shrink_factor = 0, then the region shrinks to the singleton centroid.
 * @param depth The penetration depth between the finger tip sphere and the
 * brick.
 */
void AddFingerTipInContactWithBrickFaceConstraint(
    const GripperBrickHelper<double>& gripper_brick_system, Finger finger,
    BrickFace brick_face, solvers::MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    systems::Context<double>* plant_context, double face_shrink_factor,
    double depth);

Eigen::Vector3d ComputeFingerTipInBrickFrame(
    const GripperBrickHelper<double>& gripper_brick, const Finger finger,
    const systems::Context<double>& plant_context,
    const Eigen::Ref<const Eigen::VectorXd>& q);
Vector3<AutoDiffXd> ComputeFingerTipInBrickFrame(
    const GripperBrickHelper<double>& gripper_brick, const Finger finger,
    const systems::Context<double>& plant_context,
    const Eigen::Ref<const AutoDiffVecXd>& q);

/**
 * Impose a kinematic constraint which prohibits the fingertip sphere from
 * sliding on the brick's face. That is, the fingertip sphere is only allowed to
 * roll on the brick's surface. Note that zero rolling (i.e., sticking) is also
 * allowed.
 * @param gripper_brick Contains the gripper brick diagram.
 * @param finger The finger that should not slide.
 * @param face The brick face that the finger sticks to (or rolls on).
 * @param rolling_angle_bound The non-negative bound on the rolling angle (in
 * radians).
 * @param prog The optimization program to which the constraint is added.
 * @param from_context The context that contains the posture of the plant,
 * before rolling occurs.
 * @param to_context The context that contains the posture of the plant, after
 * rolling occurs.
 * @param q_from The variable representing the "from" posture.
 * @param q_to The variable representing the "to" posture.
 */
void AddFingerNoSlidingConstraint(
    const GripperBrickHelper<double>& gripper_brick, Finger finger,
    BrickFace face, double rolling_angle_bound,
    solvers::MathematicalProgram* prog, systems::Context<double>* from_context,
    systems::Context<double>* to_context,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_from,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_to,
    double face_shrink_factor, double depth);

/**
 * Impose the kinematic constraint that the finger can only roll (or stick)
 * starting from a given fixed posture. The angle of the finger is defined as
 * the pitch angle of the finger link 2 (about +x axis). The change of finger
 * angles from the starting posture to the ending posture is limited to be
 * within rolling_angle_upper and rolling_angle_lower. If you want to impose
 * sticking contact (no rolling), then set rolling_angle_lower =
 * rolling_angle_upper = 0.
 * @param gripper_brick Contains the gripper brick diagram.
 * @param finger The finger that should not slide.
 * @param face The brick face that the finger sticks to (or rolls on).
 * @param rolling_angle_lower The lower bound on the rolling angle.
 * @param rolling_angle_upper The upper bound on the rolling angle.
 * @param prog The optimization program to which the constraint is added.
 * @param from_context The context containing posture, from which the finger
 * should stick to (or roll).
 * @param to_context The context that contains the value of the posture after
 * rolling (sticking).
 * @param q_to The variable representing the "to" posture.
 * @param face_shrink_factor A factor < 1, indicates the region on the face that
 * the finger can roll within.
 */
void AddFingerNoSlidingFromFixedPostureConstraint(
    const GripperBrickHelper<double>& gripper_brick, Finger finger,
    BrickFace face, double rolling_angle_lower, double rolling_angle_upper,
    solvers::MathematicalProgram* prog,
    const systems::Context<double>& from_fixed_context,
    systems::Context<double>* to_context,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_to,
    double face_shrink_factor, double depth);

namespace internal {
/**
 * Impose the constraint that the finger rolls along the line (coinciding with
 * the face).
 * The formulation of the constraint is
 *
 *     p_translation_to - p_translation_from - radius * (θ_to - θ_from) = 0
 * where θ_to and θ_from are the pitch angle of the finger tip in the "from"
 * and "to" postures respectively.
 * The variables are (q_from, q_to).
 * This constraint only has 1 row. It only constrains that in the tangential
 * direction along the brick surface, the translation of the finger tip matches
 * with the rolling angle. This constraint does NOT constrain that in the
 * normal direction, the finger remains in contact. The user should impose
 * the constraint in the normal direction separately.
 */
class FingerNoSlidingConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FingerNoSlidingConstraint)

  FingerNoSlidingConstraint(const GripperBrickHelper<double>* gripper_brick,
                            Finger finger, BrickFace face,
                            systems::Context<double>* from_context,
                            systems::Context<double>* to_context);

  ~FingerNoSlidingConstraint() override {}

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "FingerNoSlidingConstraint::Eval doesn't support symbolic variables.");
  }
  const GripperBrickHelper<double>* gripper_brick_;
  Finger finger_;
  BrickFace face_;
  systems::Context<double>* from_context_;
  systems::Context<double>* to_context_;
};

/**
 * Same as FingerNoSlidingConstraint, except the "from posture" is fixed, so the
 * decision variables are only q_to.
 */
class FingerNoSlidingFromFixedPostureConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FingerNoSlidingFromFixedPostureConstraint)

  FingerNoSlidingFromFixedPostureConstraint(
      const GripperBrickHelper<double>* gripper_brick, Finger finger,
      BrickFace face, const systems::Context<double>* from_context,
      systems::Context<double>* to_context);

  ~FingerNoSlidingFromFixedPostureConstraint() override {}

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::runtime_error(
        "FingerNoSlidingConstraint::Eval doesn't support symbolic variables.");
  }

 private:
  const GripperBrickHelper<double>* gripper_brick_;
  Finger finger_;
  BrickFace face_;
  const systems::Context<double>* from_context_;
  systems::Context<double>* to_context_;
};
}  // namespace internal
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
