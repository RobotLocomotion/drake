/// @file
/// Utilities for arithmetic on gradients.

#pragma once

#include <Eigen/Dense>

namespace drake {
namespace math {

/*
 * Recursively defined template specifying a matrix type of the correct size for
 * a gradient of a matrix function with respect to Nq variables, of any order.
 */
template <typename Derived, int Nq, int DerivativeOrder = 1>
struct Gradient {
  typedef typename Eigen::Matrix<
      typename Derived::Scalar,
      ((Derived::SizeAtCompileTime == Eigen::Dynamic || Nq == Eigen::Dynamic)
           ? Eigen::Dynamic
           : Gradient<Derived, Nq,
                      DerivativeOrder - 1>::type::SizeAtCompileTime),
      Nq> type;
};

/*
 * Base case for recursively defined gradient template.
 */
template <typename Derived, int Nq>
struct Gradient<Derived, Nq, 1> {
  typedef typename Eigen::Matrix<typename Derived::Scalar,
                                 Derived::SizeAtCompileTime, Nq> type;
};

}  // namespace math
}  // namespace drake
