#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace perception {
namespace estimators {

/**
 * Computes estimated transform by aligning principle directions via principle
 * component analysis (PCA).
 * @param y_W A 3xn matrix represent `n` measured points, fixed in body `B`
 * (inferred via PCA), expressed in world coordinates.
 * @param X_WB The pose of the inferred shape in the world frame.
 */
Eigen::Isometry3d EstimatePcaBodyPose(const Eigen::Matrix3Xd& y_W);

/**
 * Computes the pose of a body described by points `p` (expressed in the body
 * frame), given observed points `y` (expressed in the world frame).
 * This requires that all correspondences be known. This performs the
 * optimization:
 *    min  ∑ᵢ |X_WB * p_Bᵢ - y_Wᵢ|²
 *   X_WB
 * which is implemented as a widely-used simplification of the quaternion-based
 * formulation presented in:
 *    Besl, Paul J., and Neil D. McKay. "A method for registration of 3-D
 *    shapes." IEEE Transactions on pattern analysis and machine intelligence
 *    14.2 (1992): 239-256.
 * @param p_B A 3xn matrix representing `n` points, fixed and expressed in the
 * body frame, {p_Bᵢ}ᵢ.
 * @param y_W A 3xn matrix representing `n` measured points of the body,
 * fixed in the body frame and expressed in the world frame, {y_Wᵢ}ᵢ,
 * where y_Wᵢ corresponds to p_Bᵢ.
 * @param X_WB Transformation from `B` to `W`, i.e., the pose of body `B` in
 * the world frame.
 */
Eigen::Isometry3d ComputeSvdBodyPose(const Eigen::Matrix3Xd& p_B,
                                     const Eigen::Matrix3Xd& y_W);

}  // namespace estimators
}  // namespace perception
}  // namespace drake
