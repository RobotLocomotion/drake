#pragma once

#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
}

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
// TODO(siyuan.feng@tri): These should go in RigidBodyTree eventually.

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
Eigen::Vector6d GetTaskSpaceVel(
    const RigidBodyTree& r, const KinematicsCache<double>& cache,
    const RigidBody& body,
    const Eigen::Vector3d& local_offset = Eigen::Vector3d::Zero());

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
Eigen::MatrixXd GetTaskSpaceJacobian(
    const RigidBodyTree& r, const KinematicsCache<double>& cache,
    const RigidBody& body,
    const Eigen::Vector3d& local_offset = Eigen::Vector3d::Zero());

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
Eigen::Vector6d GetTaskSpaceJacobianDotTimesV(
    const RigidBodyTree& r, const KinematicsCache<double>& cache,
    const RigidBody& body,
    const Eigen::Vector3d& local_offset = Eigen::Vector3d::Zero());

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
