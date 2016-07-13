#pragma once

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"

using namespace Eigen;

// TODO(siyuan.feng@tri): These should go in RigidBodyTree eventually.

/**
 * @brief This function computes the task space velocity of a frame attached
 * to the frame identified by body_or_frame_id with a local_offset in that
 * frame.
 * @param r reference to the RigidBodyTree
 * @param cache computed kinematic cache
 * @param body_or_frame_id can be either the body id or frame id
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space velocity
 */
typedef Matrix<double, 6, 1> Vector6d;
Vector6d GetTaskSpaceVel(const RigidBodyTree& r,
                         const KinematicsCache<double>& cache,
                         int body_or_frame_id,
                         const Vector3d& local_offset = Vector3d::Zero());

/**
 * @brief This function computes the task space Jacobian of a frame attached
 * to the frame identified by body_or_frame_id with a local_offset in that
 * frame.
 * @param r reference to the RigidBodyTree
 * @param cache computed kinematic cache
 * @param body_or_frame_id can be either the body id or frame id
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space Jacobian, x_dot = J * v, x_dot is task space vel, v is
 * generalized velocity.
 */
MatrixXd GetTaskSpaceJacobian(const RigidBodyTree& r,
                              const KinematicsCache<double>& cache,
                              int body_or_frame_id,
                              const Vector3d& local_offset = Vector3d::Zero());

/**
 * @brief This function computes the task space Jacobian times the generalized
 * velocity of a frame attached to the frame identified by body_or_frame_id
 * with a local_offset in that frame.
 * @param r reference to the RigidBodyTree
 * @param cache computed kinematic cache
 * @param body_or_frame_id can be either the body id or frame id
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space Jacobian dot * v, x_ddot = J * v_dot + Jdv, x_ddot is
 * task space acceleration, v_dot is generalized acceleration.
 */
Vector6d GetTaskSpaceJacobianDotTimesV(
    const RigidBodyTree& r, const KinematicsCache<double>& cache,
    int body_or_frame_id, const Vector3d& local_offset = Vector3d::Zero());
