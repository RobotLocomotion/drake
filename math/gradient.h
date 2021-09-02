/** @file
Utilities for arithmetic on gradients. */

#pragma once

#include <Eigen/Dense>

namespace drake {
namespace math {

/** Recursively defined template specifying a matrix type of the correct size
for a gradient of a matrix function with respect to `nq` variables, of any
order. */
template <typename Derived, int nq, int derivative_order = 1>
struct Gradient {
  typedef typename Eigen::Matrix<
      typename Derived::Scalar,
      ((Derived::SizeAtCompileTime == Eigen::Dynamic || nq == Eigen::Dynamic)
           ? Eigen::Dynamic
           : Gradient<Derived, nq,
                      derivative_order - 1>::type::SizeAtCompileTime),
      nq> type;
};

/** Base case for recursively defined gradient template. */
template <typename Derived, int nq>
struct Gradient<Derived, nq, 1> {
  typedef typename Eigen::Matrix<typename Derived::Scalar,
                                 Derived::SizeAtCompileTime, nq> type;
};

}  // namespace math
}  // namespace drake
