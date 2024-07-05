#pragma once

#include <array>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* Computes the the 1D base node of a point x in reference space. */
template <typename T>
int base_node(const T& x) {
  return static_cast<int>(std::floor(x + static_cast<T>(0.5)));
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

template <typename T>
struct BSplineWeights {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BSplineWeights);

  /* Weights and weight gradients in a single dimension for three neighbor grid
   nodes. */
  struct Data {
    Vector3<T> dw;
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
    const T d0 = x_reference - base_node;
    const T z = 0.5 - d0;
    const T z2 = z * z;
    result.w(0) = 0.5 * z2;
    result.w(1) = 0.75 - d0 * d0;
    const T d1 = 1.0 - d0;
    const T zz = 1.5 - d1;
    const T zz2 = zz * zz;
    result.w(2) = 0.5 * zz2;

    result.dw(0) = -z;
    result.dw(1) = -2.0 * d0;
    result.dw(2) = zz;
    return result;
  }

  static constexpr int kDim = 3;
  std::array<Data, kDim> data_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
