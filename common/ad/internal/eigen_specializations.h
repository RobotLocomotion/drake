#pragma once

#include <limits>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

/* This file contains Eigen-related specializations for Drake's AutoDiff
scalar type.

The specializations both add basic capability (e.g., NumTraits) as well as
improve performance (e.g., opt-in to rvalue moves instead of copies).

NOTE: This file should never be included directly, rather only from
auto_diff.h in a very specific order. */

#ifndef DRAKE_DOXYGEN_CXX

// An abbreviation makes this file much more readable (w.r.t. 80-col wrapping).
// We use "ADS" to stand for "AutoDiff scalar".
#define DRAKE_ADS drake::ad::AutoDiff

namespace Eigen {

// === See Eigen/src/Core/NumTraits.h ===

// https://eigen.tuxfamily.org/dox/structEigen_1_1NumTraits.html
template <>
struct NumTraits<DRAKE_ADS> : public NumTraits<double> {
  using Real = DRAKE_ADS;
  using NonInteger = DRAKE_ADS;
  using Nested = DRAKE_ADS;
  using Literal = double;
  enum {
    RequireInitialization = 1
  };
};

// Computing "ADS [op] ADS" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<DRAKE_ADS, DRAKE_ADS, BinOp> {
  using ReturnType = DRAKE_ADS;
};

// Computing "ADS [op] double" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<DRAKE_ADS, double, BinOp> {
  using ReturnType = DRAKE_ADS;
};

// Computing "double [op] ADS" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<double, DRAKE_ADS, BinOp> {
  using ReturnType = DRAKE_ADS;
};

namespace internal {

// === See Eigen/src/Core/functors/AssignmentFunctors.h ===

// This specialization allows `b` to be moved-from rather than copied.
template <>
struct assign_op<DRAKE_ADS, DRAKE_ADS> {
  // NOLINTNEXTLINE(runtime/references) to match the Eigen signature.
  void assignCoeff(DRAKE_ADS& a, const DRAKE_ADS& b) const { a = b; }
  // NOLINTNEXTLINE(runtime/references) to match the Eigen signature.
  void assignCoeff(DRAKE_ADS& a, DRAKE_ADS&& b) const { a = std::move(b); }
};

// === See Eigen/src/Core/functors/UnaryFunctors.h ===

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_opposite_op<DRAKE_ADS> {
  DRAKE_ADS operator()(DRAKE_ADS a) const { return -std::move(a); }
};

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_conjugate_op<DRAKE_ADS> {
  DRAKE_ADS operator()(const DRAKE_ADS& a) const { return a; }
  DRAKE_ADS operator()(DRAKE_ADS&& a) const { return std::move(a); }
};

// === See Eigen/src/Core/functors/BinaryFunctors.h ===

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_sum_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a += b;
    return a;
  }
};

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_product_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a *= b;
    return a;
  }
};

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_difference_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a -= b;
    return a;
  }
};

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_quotient_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a /= b;
    return a;
  }
};

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_min_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator() (DRAKE_ADS a, const DRAKE_ADS& b) const {
    return drake::ad::min(std::move(a), b);
  }
};

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_max_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator() (DRAKE_ADS a, const DRAKE_ADS& b) const {
    // NOLINTNEXTLINE(build/include_what_you_use) false positive.
    return drake::ad::max(std::move(a), b);
  }
};

// === See Eigen/src/Core/Redux.h ===

// This specialization reduces copying of `result` by adding `std::move`.
template <typename Derived>
struct redux_impl<
    scalar_sum_op<DRAKE_ADS, DRAKE_ADS>,
    Derived,
    /* Traversal = */ DefaultTraversal,
    /* Unrolling = */ NoUnrolling> {
  using Scalar = DRAKE_ADS;
  template <typename Func>
  static DRAKE_ADS run(const Derived& mat, const Func& func) {
    DRAKE_ASSERT(mat.size() > 0);
    DRAKE_ADS result = mat.coeffByOuterInner(0, 0);
    for (Index i = 1; i < mat.innerSize(); ++i)
      result = func(std::move(result), mat.coeffByOuterInner(0, i));
    for (Index i = 1; i < mat.outerSize(); ++i)
      for (Index j = 0; j < mat.innerSize(); ++j)
        result = func(std::move(result), mat.coeffByOuterInner(i, j));
    return result;
  }
};

}  // namespace internal
}  // namespace Eigen

namespace std {
template<>
class numeric_limits<DRAKE_ADS> : public numeric_limits<double> {};
}  // namespace std

#undef DRAKE_ADS

#endif  // DRAKE_DOXYGEN_CXX
