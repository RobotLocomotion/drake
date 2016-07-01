#pragma once

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"

using namespace Eigen;

// These should go in RigidBodyTree eventually.

/**
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
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space Jacobian, x_dot = J * v, x_dot is task space vel, v is
 * generalized velocity.
 */
MatrixXd GetTaskSpaceJacobian(const RigidBodyTree& r,
                              KinematicsCache<double>& cache,
                              int body_or_frame_id,
                              const Vector3d& local_offset = Vector3d::Zero());

/**
 * @param local_offset from the body frame origin to the point of interest
 * in body frame
 * @return task space Jacobian dot * v, x_ddot = J * v_dot + Jdv, x_ddot is
 * task space acceleration, v_dot is generalized acceleration.
 */
Vector6d GetTaskSpaceJacobianDotTimesV(
    const RigidBodyTree& r, KinematicsCache<double>& cache,
    int body_or_frame_id, const Vector3d& local_offset = Vector3d::Zero());
