#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
// TODO(siyuan.feng): These should go in RigidBodyTree eventually.

/**
 * This function computes the task space velocity of a frame attached
 * to @p body with @p local_offset in the body frame.
 * @param r reference to the RigidBodyTree
 * @param cache computed kinematic cache
 * @param body reference to the body
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space velocity
 */
Vector6<double> GetTaskSpaceVel(
    const RigidBodyTree<double>& r, const KinematicsCache<double>& cache,
    const RigidBody<double>& body,
    const Vector3<double>& local_offset = Vector3<double>::Zero());

/**
 * This function computes the task space Jacobian of a frame attached
 * to @p body with @p local_offset in the body frame.
 * @param r reference to the RigidBodyTree
 * @param cache computed kinematic cache
 * @param body reference to the body
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space Jacobian, x_dot = J * v, x_dot is task space vel, v is
 * generalized velocity.
 */
MatrixX<double> GetTaskSpaceJacobian(
    const RigidBodyTree<double>& r, const KinematicsCache<double>& cache,
    const RigidBody<double>& body,
    const Vector3<double>& local_offset = Vector3<double>::Zero());

/**
 * This function computes the task space Jacobian times the generalized
 * velocity of a frame attached to @p body with @p local_offset in the body
 * frame.
 * @param r reference to the RigidBodyTree
 * @param cache computed kinematic cache
 * @param body reference to the body
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space Jacobian dot * v, x_ddot = J * v_dot + Jdv, x_ddot is
 * task space acceleration, v_dot is generalized acceleration.
 */
Vector6<double> GetTaskSpaceJacobianDotTimesV(
    const RigidBodyTree<double>& r, const KinematicsCache<double>& cache,
    const RigidBody<double>& body,
    const Vector3<double>& local_offset = Vector3<double>::Zero());

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
