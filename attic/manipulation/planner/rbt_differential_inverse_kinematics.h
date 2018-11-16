#pragma once

#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace rbt {

/**
 * A wrapper over
 * DoDifferentialInverseKinematics(q_current, v_current, V, J, params)
 * that tracks frame E's spatial velocity.
 * q_current and v_current are taken from @p cache. V is computed by first
 * transforming @p V_WE to V_WE_E, then taking the element-wise product between
 * V_WE_E and the gains (specified in frame E) in @p parameters, and only
 * selecting the non zero elements. J is computed similarly.
 * @param robot Kinematic tree.
 * @param cache Kinematic cache build from the current generalized position and
 * velocity.
 * @param V_WE_desired Desired world frame spatial velocity of @p frame_E.
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const Vector6<double>& V_WE_desired, const RigidBodyFrame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

/**
 * A wrapper over
 * DoDifferentialInverseKinematics(robot, cache, V_WE_desired, frame_E, params)
 * that tracks frame E's pose in the world frame.
 * q_current and v_current are taken from @p cache. V_WE is computed by
 * ComputePoseDiffInCommonFrame(X_WE, X_WE_desired) / dt, where X_WE is computed
 * from @p cache, and dt is taken from @p parameters.
 * @param robot Robot model.
 * @param cache KinematiCache built from the current generalized position and
 * velocity.
 * @param X_WE_desired Desired pose of @p frame_E in the world frame.
 * @param frame_E End effector frame.
 * @param parameters Collection of various problem specific constraints and
 * constants.
 * @return If the solver successfully finds a solution, joint_velocities will
 * be set to v, otherwise it will be nullopt.
 */
DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const Isometry3<double>& X_WE_desired,
    const RigidBodyFrame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters);

}  // namespace rbt
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
