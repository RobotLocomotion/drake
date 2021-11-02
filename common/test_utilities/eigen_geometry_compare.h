#pragma once

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_deprecated.h"

namespace drake {

/**
 * Expects that a rotation matrix belongs to SO(3):
 *    R ∈ SO(3) =>
 *      Rᵀ R = I (orthonormal)
 *      det(R) = 1 (right-hand rule)
 * @param R Rotation matrix.
 * @param tolerance  The tolerance for determining equivalence for
 * the orthonormal and right-hand rule cases.
 */
DRAKE_DEPRECATED("2022-02-01", "This function is being removed.")
[[nodiscard]] ::testing::AssertionResult ExpectRotMat(
    const Eigen::Matrix3d& R, double tolerance);

/**
 * Compares two SE(3) Transforms.
 * @param X_expected Expected SE(3) transform.
 * @param X_actual Actual SE(3) transform.
 * @param tolerance The tolerance for determining equivalence for the
 * rotation matrix cases (using ExpectRotMat) and comparing both transforms
 * as matrices.
 */
DRAKE_DEPRECATED("2022-02-01", "This function is being removed.")
[[nodiscard]]::testing::AssertionResult CompareTransforms(
    const Eigen::Isometry3d& X_expected, const Eigen::Isometry3d& X_actual,
    double tolerance);

}   // namespace drake
