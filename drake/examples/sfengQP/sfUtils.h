#pragma once

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"

using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
Vector6d getTaskSpaceVel(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset = Vector3d::Zero());
MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r, KinematicsCache<double> &cache, int body, const Vector3d &local_offset = Vector3d::Zero());
Vector6d getTaskSpaceJacobianDotTimesV(const RigidBodyTree &r, KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset = Vector3d::Zero());
