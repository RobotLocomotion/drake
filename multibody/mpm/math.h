#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/mpm/simd_scalar.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* Returns (i, j, k)th entry of the third order permutation tensor.
 See https://en.wikipedia.org/wiki/Levi-Civita_symbol for details.
 @pre i, j, k ∈ {0, 1, 2} */
inline double LeviCivita(int i, int j, int k) {
  static const double lookup_table[3][3][3] = {
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0}},
      {{0.0, 0.0, -1.0}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

  return lookup_table[i][j][k];
}

/* Computes A: εwhere ε is the Levi-Civita tensor. */
template <typename T>
Vector3<T> ContractWithLeviCivita(const Matrix3<T>& A) {
  Vector3<T> A_dot_eps = {0.0, 0.0, 0.0};
  A_dot_eps(0) = A(1, 2) - A(2, 1);
  A_dot_eps(1) = A(2, 0) - A(0, 2);
  A_dot_eps(2) = A(0, 1) - A(1, 0);
  return A_dot_eps;
}

// TODO(xuchenhan-tri): Move this into a separate file.
template <typename T>
struct MassAndMomentum {
  T mass{0.0};
  Vector3<T> linear_momentum{Vector3<T>::Zero()};
  Vector3<T> angular_momentum{Vector3<T>::Zero()};
};

/* Computes the the 1D base node of a point x in reference space. */
template <typename T>
int base_node(const T& x) {
  return static_cast<int>(std::floor(x + static_cast<T>(0.5)));
}

template <>
inline int base_node(const SimdScalar<double>& x) {
  double x0 = x.get_lane();
  return static_cast<int>(std::floor(x0 + static_cast<double>(0.5)));
}

template <>
inline int base_node(const SimdScalar<float>& x) {
  float x0 = x.get_lane();
  return static_cast<int>(std::floor(x0 + static_cast<float>(0.5)));
}

/* Computes the the 3D base node of a point x reference space. */
template <typename T>
Vector3<int> base_node(const Vector3<T>& x) {
  Vector3<int> result;
  result[0] = base_node(x[0]);
  result[1] = base_node(x[1]);
  result[2] = base_node(x[2]);
  return result;
}

// TODO(xuchenhan-tri): MLS-MPM doesn't need weight gradients.
template <typename T>
struct BSplineWeights {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BSplineWeights);

  /* Weights and weight gradients in a single dimension for three neighbor grid
   nodes. */
  struct Data {
    Vector3<T> w;
  };

  BSplineWeights(const Vector3<T>& x, const T& dx) {
    const Vector3<T> x_reference = x / dx;
    const Vector3<int> base_node = mpm::internal::base_node(x_reference);
    for (int d = 0; d < 3; ++d) {
      data_[d] = Compute(x_reference[d], base_node[d]);
    }
  }

  const std::array<Data, 3>& data() const { return data_; }

  T weight(int i, int j, int k) const {
    return data_[0].w(i) * data_[1].w(j) * data_[2].w(k);
  }

 private:
  /* Computes the weight and weight gradients in a single dimension.
   @param[in] x_reference  The particle position in the reference frame.
   @param[in] base_node    The base grid node position in reference frame. */
  Data Compute(const T& x_reference, int base_node) const {
    Data result;
    const T d0 = x_reference - static_cast<T>(base_node);
    const T z = static_cast<T>(0.5) - d0;
    const T z2 = z * z;
    result.w(0) = static_cast<T>(0.5) * z2;
    result.w(1) = static_cast<T>(0.75) - d0 * d0;
    const T d1 = static_cast<T>(1.0) - d0;
    const T zz = static_cast<T>(1.5) - d1;
    const T zz2 = zz * zz;
    result.w(2) = static_cast<T>(0.5) * zz2;

    return result;
  }

  static constexpr int kDim = 3;
  std::array<Data, kDim> data_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
