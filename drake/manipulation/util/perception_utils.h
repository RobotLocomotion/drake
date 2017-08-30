#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace manipulation {
namespace util {

/**
 * Converts a 7 dimensional VectorX<double> describing a pose (composed by
 * positions in the first 3 dimensions and orientation in quaternions in the
 * next 4) into an Eigen::Isometry3d object.
 * @param pose_vector input pose as a VectorX<double>.
 * @return An Eigen::Isometry3d with the output pose.
 */
Eigen::Isometry3d VectorToIsometry3d(const VectorX<double> &pose_vector);

/**
 * Converts a pose specified as an Eigen::Isometry3d into a 7 dimensional
 * VectorX<double> (composed by positions in the first 3 dimensions and
 * orientation in quaternions in the next 4).
 * @param pose input as an Eigen::Isometry3d.
 * @return An output pose as a 7 dimensional VectorX<double>.
 */
VectorX<double> Isometry3dToVector(const Eigen::Isometry3d& pose);

}  // namespace util
}  // namespace manipulation
}  // namespace drake