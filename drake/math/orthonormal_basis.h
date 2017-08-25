#pragma once

#include <limits>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Creates a right-handed local basis from a given axis. Defines two other
/// arbitrary axes such that the basis is orthonormal. The basis is R_WL, where
/// W is the frame in which the input axis is expressed and L is a local basis
/// such that v_W = R_WL * v_L.
///
/// @param[in] axis_index The index of the axis (in the range [0,2]), with
///                       0 corresponding to the x-axis, 1 corresponding to the
///                       y-axis, and z-corresponding to the z-axis.
/// @param[in] axis_W   The vector defining the basis's given axis expressed
///                     in frame W.
/// @retval R_WL        The computed basis.
/// @throws std::logic_error if @p axis_W is not a unit vector or @p axis_index
///         does not lie in the range [0,2].
template <class T>
Matrix3<T> ComputeBasisFromAxis(int axis_index, const Vector3<T>& axis_W) {
  using std::abs;

  // Verify that the correct axis is given.
  if (axis_index < 0 || axis_index > 2)
    throw std::logic_error("Invalid axis specified: must be 0, 1, or 2.");

  // Verify that the vector is normalized.
  const T nrm_sq = axis_W.squaredNorm();
  if (abs(nrm_sq - 1.0) > 10 * std::numeric_limits<double>::epsilon())
    throw std::logic_error("Vector does not appear to be normalized.");

  // The axis corresponding to the smallest component of axis_W will be *most*
  // perpendicular.
  const Vector3<T> u(axis_W.cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);
  Vector3<T> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);

  // Now define additional vectors in the basis.
  Vector3<T> v1_W = axis_W.cross(perpAxis).normalized();
  Vector3<T> v2_W = axis_W.cross(v1_W);

  // Set the columns of the matrix.
  Matrix3<T> R_WL;
  R_WL.col(axis_index) = axis_W;
  R_WL.col((axis_index + 1) % 3) = v1_W;
  R_WL.col((axis_index + 2) % 3) = v2_W;

  return R_WL;
}

}  // namespace math
}  // namespace drake
