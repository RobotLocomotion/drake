#pragma once

#include <limits>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Computes an orthonormal basis { ii, jj, kk } from provided vector @p ii
/// such that ii x jj = kk (with a right-handed cross product).
/// @param[in/out] ii      A (possibly unnormalized) vector; on return, a
///                        normalized vector.
/// @param[out] jj  On return, a second vector in the basis.
/// @param[out] kk  On return, the third vector in the basis.
/// @throws std::logic_error if @p ii, @p jj, or @p kk is null or @p ii is zero.
/// @warning
template <class T>
void CalcOrthonormalBasis(Vector3<T>* ii, Vector3<T>* jj, Vector3<T>* kk) {
  if (!ii)
    throw std::logic_error("ii vector is null.");
  if (!jj)
    throw std::logic_error("jj vector is null.");
  if (!kk)
    throw std::logic_error("kk vector is null.");

  // Normalize ii.
  T ii_nrm = ii->norm();
  if (ii_nrm < std::numeric_limits<double>::epsilon()) {
    throw std::logic_error("ii vector is zero.");
  }
  (*ii) /= ii_nrm;

  const Vector3<T> u(ii->cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);

  // The axis corresponding to the smallest component of ii will be *most*
  // perpendicular.
  Vector3<T> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);

  // Now define jj and kk.
  *jj = ii->cross(perpAxis).normalized();
  *kk = ii->cross(*jj);
}

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
  const T nrm = z_axis_W.norm();
  if (abs(nrm - 1.0) > 10 * std::numeric_limits<double>::epsilon())
    throw std::logic_error("z-axis vector does not appear to be normalized.");
  Vector3<T> v = z_axis_W;

  // Calculate the other two vectors.
  Vector3<T> v1, v2;
  CalcOrthonormalBasis(&v, &v1, &v2);

  // Set the columns of the matrix: z x v1 = v2 means that v1 should be x and
  // v2 should be y.
  Matrix3<T> R_WL;
  R_WL.col(0) = v1;
  R_WL.col(1) = v2;
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
  const T nrm = x_axis_W.norm();
  if (abs(nrm - 1.0) > 10 * std::numeric_limits<double>::epsilon())
    throw std::logic_error("z-axis vector does not appear to be normalized.");
  Vector3<T> v = x_axis_W;

  // Calculate the other two vectors.
  Vector3<T> v1, v2;
  CalcOrthonormalBasis(&v, &v1, &v2);

  // Set the columns of the matrix: x × v1 = v2 means that v1 should be y and
  // v2 should be z.
  Matrix3<T> R_WL;
  R_WL.col(0) = x_axis_W;
  R_WL.col(1) = v1;
  R_WL.col(2) = v2;

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
  const T nrm = y_axis_W.norm();
  if (abs(nrm - 1.0) > 10 * std::numeric_limits<double>::epsilon())
    throw std::logic_error("z-axis vector does not appear to be normalized.");
  Vector3<T> v = y_axis_W;

  // Calculate the other two vectors.
  Vector3<T> v1, v2;
  CalcOrthonormalBasis(&v, &v1, &v2);

  // Set the columns of the matrix: y × v1 = v2 means that v1 should be z and
  // v2 should be x.
  Matrix3<T> R_WL;
  R_WL.col(0) = v2;
  R_WL.col(1) = y_axis_W;
  R_WL.col(2) = v1;

  return R_WL;
}

}  // namespace math
}  // namespace drake
