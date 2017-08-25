#pragma once

#include <limits>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Creates a right-handed local basis from a z-axis. Defines an arbitrary x-
/// and y-axis such that the basis is orthonormal. The basis is R_WL, where W
/// is the frame in which the z-axis is expressed and L is a local basis such
/// that v_W = R_WL * v_L.
///
/// @param[in] z_axis_W   The vector defining the basis's z-axis expressed
///                       in frame W.
/// @retval R_WL          The computed basis.
/// @throws std::logic_error if z_axis_W is not a unit vector.
template <class T>
Matrix3<T> ComputeBasisFromZ(const Vector3<T>& z_axis_W) {
  using std::abs;

  // Verify that the vector is normalized.
  const T nrm = z_axis_W.squaredNorm();
  if (abs(nrm - 1.0) > 10 * std::numeric_limits<double>::epsilon())
    throw std::logic_error("z-axis vector does not appear to be normalized.");

  // The axis corresponding to the smallest component of z_axis_W will be *most*
  // perpendicular.
  const Vector3<T> u(z_axis_W.cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);
  Vector3<T> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);

  // Now define x- and y-axes.
  Vector3<T> x_axis_W = z_axis_W.cross(perpAxis).normalized();
  Vector3<T> y_axis_W = z_axis_W.cross(x_axis_W);

  // Set the columns of the matrix.
  Matrix3<T> R_WL;
  R_WL.col(0) = x_axis_W;
  R_WL.col(1) = y_axis_W;
  R_WL.col(2) = z_axis_W;

  return R_WL;
}

/// Creates a right-handed local basis from an x-axis. Defines an arbitrary z-
/// and y-axis such that the basis is orthonormal. The basis is R_WL, where W
/// is the frame in which the x-axis is expressed and L is a local basis such
/// that v_W = R_WL * v_L.
///
/// @param[in] x_axis_W   The vector defining the basis's x-axis expressed
///                       in frame W.
/// @retval R_WL          The computed basis.
/// @throws std::logic_error if x_axis_W is not a unit vector.
template <class T>
Matrix3<T> ComputeBasisFromX(const Vector3<T>& x_axis_W) {
  using std::abs;

  // Verify that the vector is normalized.
  const T nrm = x_axis_W.squaredNorm();
  if (abs(nrm - 1.0) > 10 * std::numeric_limits<double>::epsilon())
    throw std::logic_error("x-axis vector does not appear to be normalized.");

  // The axis corresponding to the smallest component of x_axis_W will be *most*
  // perpendicular.
  const Vector3<T> u(x_axis_W.cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);
  Vector3<T> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);

  // Now define y- and z-axes.
  Vector3<T> y_axis_W = x_axis_W.cross(perpAxis).normalized();
  Vector3<T> z_axis_W = x_axis_W.cross(y_axis_W);

  // Set the columns of the matrix.
  Matrix3<T> R_WL;
  R_WL.col(0) = x_axis_W;
  R_WL.col(1) = y_axis_W;
  R_WL.col(2) = z_axis_W;

  return R_WL;
}

/// Creates a right-handed local basis from a y-axis. Defines an arbitrary x-
/// and z-axis such that the basis is orthonormal. The basis is R_WL, where W
/// is the frame in which the y-axis is expressed and L is a local basis such
/// that v_W = R_WL * v_L.
///
/// @param[in] y_axis_W   The vector defining the basis's y-axis expressed
///                       in frame W.
/// @retval R_WL          The computed basis.
/// @throws std::logic_error if y_axis_W is not a unit vector.
template <class T>
Matrix3<T> ComputeBasisFromY(const Vector3<T>& y_axis_W) {
  using std::abs;

  // Verify that the vector is normalized.
  const T nrm = y_axis_W.squaredNorm();
  if (abs(nrm - 1.0) > 10 * std::numeric_limits<double>::epsilon())
    throw std::logic_error("y-axis vector does not appear to be normalized.");

  // The axis corresponding to the smallest component of y_axis_W will be *most*
  // perpendicular.
  const Vector3<T> u(y_axis_W.cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);
  Vector3<T> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);

  // Now define x- and z-axes.
  Vector3<T> z_axis_W = y_axis_W.cross(perpAxis).normalized();
  Vector3<T> x_axis_W = y_axis_W.cross(z_axis_W);

  // Set the columns of the matrix.
  Matrix3<T> R_WL;
  R_WL.col(0) = x_axis_W;
  R_WL.col(1) = y_axis_W;
  R_WL.col(2) = z_axis_W;

  return R_WL;
}

}  // namespace math
}  // namespace drake
