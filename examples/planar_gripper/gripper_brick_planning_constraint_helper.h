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
 * @param prog The program to which the constraint is added.
 */
template <typename T>
void AddFrictionConeConstraint(
    const GripperBrickHelper<T>& gripper_brick, Finger finger,
    BrickFace brick_face,
    const Eigen::Ref<const Vector2<symbolic::Variable>>& f_Cb_B,
    solvers::MathematicalProgram* prog);

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
 * brick. @default to 1mm.
 */
void AddFingerTipInContactWithBrickFaceConstraint(
    const GripperBrickHelper<double>& gripper_brick_system, Finger finger,
    BrickFace brick_face, solvers::MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    systems::Context<double>* plant_context, double face_shrink_factor,
    double depth = 1e-3);

Eigen::Vector3d ComputeFingerTipInBrickFrame(
    const GripperBrickHelper<double>& gripper_brick, const Finger finger,
    const systems::Context<double>& plant_context,
    const Eigen::Ref<const Eigen::VectorXd>& q);
Vector3<AutoDiffXd> ComputeFingerTipInBrickFrame(
    const GripperBrickHelper<double>& gripper_brick, const Finger finger,
    const systems::Context<double>& plant_context,
    const Eigen::Ref<const AutoDiffVecXd>& q);
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
