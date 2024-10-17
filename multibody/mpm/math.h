#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/mpm/simd_scalar.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* Computes A: ε where ε is the Levi-Civita tensor. */
template <typename T>
Vector3<T> ContractWithLeviCivita(const Matrix3<T>& A) {
  Vector3<T> A_dot_eps = {0.0, 0.0, 0.0};
  A_dot_eps(0) = A(1, 2) - A(2, 1);
  A_dot_eps(1) = A(2, 0) - A(0, 2);
  A_dot_eps(2) = A(0, 1) - A(1, 0);
  return A_dot_eps;
}

// TODO(xuchenhan-tri): Move this into a separate file.
/* MassAndMomentum stores the mass, linear momentum, and angular momentum of an
 object, represented by a collection of particles or a grid. */
template <typename T>
struct MassAndMomentum {
  T mass{0.0};
  Vector3<T> linear_momentum{Vector3<T>::Zero()};
  Vector3<T> angular_momentum{Vector3<T>::Zero()};
};

/* Computes the the 1D base node of a point x in reference space.
 @tparam double or float. */
template <typename T>
int ComputeBaseNode(const T& x) {
  return static_cast<int>(std::floor(x + static_cast<T>(0.5)));
}

/* Computes the the 1D base node of a few points in reference space.
 @param x A vector of points in reference space.
 @pre all points in `x` are have the same base node.
 @tparam double or float. */
template <typename T>
inline int ComputeBaseNode(const SimdScalar<T>& x) {
  T x0 = x.get_lane();
  return static_cast<int>(std::floor(x0 + static_cast<T>(0.5)));
}

/* Computes the the 3D base node of a point x reference space.
 @tparam double, float, SimdScalar<double>, or SimdScalar<float> */
template <typename T>
Vector3<int> ComputeBaseNode(const Vector3<T>& x) {
  Vector3<int> result;
  result[0] = ComputeBaseNode(x[0]);
  result[1] = ComputeBaseNode(x[1]);
  result[2] = ComputeBaseNode(x[2]);
  return result;
}

/* BsplineWeights computes and stores the weights between a particle and the
grid nodes in its support in the cartesian grid with grid spacing dx (meter).
The weights are computed using the quadratic B-spline kernel, and each particle
has 3x3x3=27 grid nodes in its support and the weights can be queried with the
`weight()` function.
@tparam double, float, Simdscalar<double>, or Simdscalar<float>. */
template <typename T>
struct BsplineWeights {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineWeights);

  /* Constructs the BsplineWeights between a particle with world frame positions
   x and the grid nodes in its support in the cartesian grid with grid spacing
   dx (meter).
   @pre if T is a SimdScalar, all particles in x have the same base node. */
  BsplineWeights(const Vector3<T>& x, const T& dx) {
    const Vector3<T> x_reference = x / dx;
    const Vector3<int> base_node = ComputeBaseNode(x_reference);
    for (int d = 0; d < 3; ++d) {
      data_[d] = Compute(x_reference[d], base_node[d]);
    }
  }

  /* Returns between the particle and the ijk-th grid node in its support.
   @pre 0 <= i, j, k < 3. */
  T weight(int i, int j, int k) const {
    return data_[0](i) * data_[1](j) * data_[2](k);
  }

 private:
  /* Computes the weights in a single dimension.
   @param[in] x_reference  The particle position in the reference frame.
   @param[in] base_node    The base grid node position in reference frame.
   @returns the weights between the particle and the three grid nodes in its
   support in a single dimension. */
  Vector3<T> Compute(const T& x_reference, int base_node) const {
    Vector3<T> result;
    const T d0 = x_reference - static_cast<T>(base_node);
    const T z = static_cast<T>(0.5) - d0;
    const T z2 = z * z;
    result(0) = static_cast<T>(0.5) * z2;
    result(1) = static_cast<T>(0.75) - d0 * d0;
    const T d1 = static_cast<T>(1.0) - d0;
    const T zz = static_cast<T>(1.5) - d1;
    const T zz2 = zz * zz;
    result(2) = static_cast<T>(0.5) * zz2;

    return result;
  }

  static constexpr int kDim = 3;
  std::array<Vector3<T>, kDim> data_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
