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
 * @return X_WB The pose of the inferred shape in the world frame.
 */
Eigen::Isometry3d ComputePcaBodyPose(const Eigen::Matrix3Xd& y_W);

/**
 * Computes the pose of a body described by points `p` (expressed in the body
 * frame), given observed points `y` (expressed in the world frame).
 * This requires that all correspondences be known. This performs the
 * optimization:
 *    min  ∑ᵢ |X_WB * p_Bᵢ - y_Wᵢ|²
 *   X_WB
 * which is implemented as a widely-used simplification of the quaternion-based
 * formulation presented in:
 *    Arun, K. Somani, Thomas S. Huang, and Steven D. Blostein. "Least-squares
 *    fitting of two 3-D point sets." IEEE Transactions on pattern analysis and
 *    machine intelligence 5 (1987): 698-700.
 * @param p_B A 3xn matrix representing `n` points, fixed and expressed in the
 * body frame, {p_Bᵢ}ᵢ.
 * @param y_W A 3xn matrix representing `n` measured points of the body,
 * fixed in the body frame and expressed in the world frame, {y_Wᵢ}ᵢ,
 * where y_Wᵢ corresponds to p_Bᵢ.
 * @return X_WB Transformation from `B` to `W`, i.e., the pose of body `B` in
 * the world frame.
 *
 * @pre p_B The body points (and ultimately y_W, barring noise) are not
 * colinear. See Sec. IV, possibility 3.
 * @note Be wary of the Degeneracy in Sec. V., when the noise in y_W is large
 * and may yield a reflected R_WB.
 */
Eigen::Isometry3d ComputeSvdBodyPose(const Eigen::Matrix3Xd& p_B,
                                     const Eigen::Matrix3Xd& y_W);

}  // namespace estimators
}  // namespace perception
}  // namespace drake
