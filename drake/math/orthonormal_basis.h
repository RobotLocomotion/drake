#pragma once

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

}  // namespace math
}  // namespace drake
