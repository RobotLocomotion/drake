#pragma once

#include <array>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* Computes the the 1D base node of a point x in reference space.
 In the 1D case, the base node, in reference space, is the closest integer to x.
 We bias upwards if x is exactly halfway between two integers.
 @tparam T Scalar type; can be double or float. */
template <typename T>
int ComputeBaseNode(const T& x) {
  /* We want to consistently bias upwards at halfway between two integers (for
   transition invariance), so we don't use std::round here. This choice also
   simplifies the computation of B-spline weights below. */
  return static_cast<int>(std::floor(x + static_cast<T>(0.5)));
}

/* Computes the the 3D base node of a point x reference space by stacking the
 results of the 1D case in each dimension.
 @tparam T Scalar type; can be double or float. */
template <typename T>
Vector3<int> ComputeBaseNode(const Vector3<T>& x) {
  return Vector3<int>(ComputeBaseNode(x[0]), ComputeBaseNode(x[1]),
                      ComputeBaseNode(x[2]));
}

/* BsplineWeights computes and stores the weights between a point x and the
 grid nodes in its support in the cartesian grid with grid spacing dx (meter).
 The weight between the point and a grid node is the product of the weights in
 each dimension. In a single dimension, the weights are computed using the
 quadratic B-spline kernel. The kernel is defined as:

    N(d) = 0.75 - d²,           if 0 <= |d| <= 0.5     (1)
           0.5 * (1.5 - |d|)²,  if 0.5 < |d| <= 1.5    (2)

 where d is the signed offset from a nearby grid node, in units of the grid
 spacing dx. The kernel is non-zero in the interval (-1.5, 1.5).

 As a result, the kernel has a support of 3 grid nodes in each dimension, so
 each point has 3x3x3=27 grid nodes in its support. The weights can be queried
 with the `weight()` function.

 @tparam T Scalar type; can be double or float. */
template <typename T>
class BsplineWeights {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineWeights);

  /* Constructs the BsplineWeights between a point with world frame positions
   x and the grid nodes in its support in the cartesian grid with grid spacing
   dx (meter). */
  BsplineWeights(const Vector3<T>& x, const T& dx) {
    const Vector3<T> x_reference = x / dx;  // Put x in the reference frame.
    const Vector3<int> base_node = ComputeBaseNode(x_reference);
    ComputeWeights(x_reference[0], base_node[0], &data_[0]);
    ComputeWeights(x_reference[1], base_node[1], &data_[1]);
    ComputeWeights(x_reference[2], base_node[2], &data_[2]);
  }

  /* Returns the weight between the point and the ijk-th grid node in its
   support.
   @note B-splines form a partition of unity, i.e., the sum of all legal
   weights equals 1.
   @pre 0 <= i, j, k < 3. */
  T weight(int i, int j, int k) const {
    return data_[0](i) * data_[1](j) * data_[2](k);
  }

 private:
  /* Computes the weights in a single dimension.
   @param[in] x_reference  The point position in the reference frame.
   @param[in] base_node    The base grid node position in reference frame.
   @param[out] result      the weights between the point and the three grid
                           nodes in its support in a single dimension. */
  void ComputeWeights(const T& x_reference, int base_node,
                      Vector3<T>* result) const {
    /* d, as in the class documentation, is the grid node's position in point
     x's reference frame. */
    const T d = static_cast<T>(base_node) - x_reference;
    /* 1-d is the _distance_ between the left most grid node and point x. */
    const T d1 = static_cast<T>(0.5) + d;          // d1 = 1.5 - (1 - d).
    (*result)(0) = static_cast<T>(0.5) * d1 * d1;  // using eq(2).
    (*result)(1) = static_cast<T>(0.75) - d * d;   // using eq(1).
    /* 1+d is the _distance_ between the right most grid node and point x. */
    const T d2 = static_cast<T>(0.5) - d;          // d2 = 1.5 - (1 + d).
    (*result)(2) = static_cast<T>(0.5) * d2 * d2;  // using eq(2).
  }

  static constexpr int kDim = 3;
  std::array<Vector3<T>, kDim> data_;
};

/* Constructs a double BsplineWeights with the given autodiff point position x
 and grid space dx (meter).
 @note This function is used for test purposes only.
 @pre x has no derivatives. */
BsplineWeights<double> MakeBsplineWeights(const Vector3<AutoDiffXd>& x,
                                          double dx);

/* Constructs a BsplineWeights.
 @tparam T Scalar type; can be double or float. */
template <typename T>
BsplineWeights<T> MakeBsplineWeights(const Vector3<T>& x, T dx) {
  return BsplineWeights<T>(x, dx);
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
