#pragma once

#include <limits>

#include <Eigen/Core>

/* This file contains Eigen-related specializations for Drake's SimdScalar
scalar type (plus the intimately related std::numeric_limits specialization).

NOTE: This file should never be included directly, rather only from
simd_scalar.h in a very specific order. */

#ifndef DRAKE_DOXYGEN_CXX

namespace std {
template <typename T>
class numeric_limits<drake::multibody::mpm::internal::SimdScalar<T>>
    : public numeric_limits<double> {};
}  // namespace std

namespace Eigen {

// === See Eigen/src/Core/NumTraits.h ===

// See https://eigen.tuxfamily.org/dox/structEigen_1_1NumTraits.html.
// We'll Inherit the constants from `T`, but be sure to fatten a few types
// up to full SimdScalar where necessary.
// @tparam T double or float.
template <typename T>
struct NumTraits<drake::multibody::mpm::internal::SimdScalar<T>>
    : public NumTraits<T> {
  // This refers to the "real part" of a complex number (e.g., std::complex).
  // Because we're not a complex number, it's just the same type as ourselves.
  using Real = drake::multibody::mpm::internal::SimdScalar<T>;

  // This promotes integer types during operations like quotients, square roots,
  // etc. We're already floating-point, so it's just the same type as ourselves.
  using NonInteger = drake::multibody::mpm::internal::SimdScalar<T>;

  // Eigen says "If you don't know what this means, just use [your type] here."
  using Nested = drake::multibody::mpm::internal::SimdScalar<T>;

  // Our constructor is required during matrix storage initialization.
  enum { RequireInitialization = 1 };
};

// Computing "SimdScalar [op] double" yields an SimdScalar.
template <typename BinOp, typename T>
struct ScalarBinaryOpTraits<drake::multibody::mpm::internal::SimdScalar<T>,
                            double, BinOp> {
  using ReturnType = drake::multibody::mpm::internal::SimdScalar<T>;
};

// Computing "double [op] SimdScalar" yields an SimdScalar.
template <typename BinOp, typename T>
struct ScalarBinaryOpTraits<
    double, drake::multibody::mpm::internal::SimdScalar<T>, BinOp> {
  using ReturnType = drake::multibody::mpm::internal::SimdScalar<T>;
};

// Computing "SimdScalar [op] float" yields an SimdScalar.
template <typename BinOp, typename T>
struct ScalarBinaryOpTraits<drake::multibody::mpm::internal::SimdScalar<T>,
                            float, BinOp> {
  using ReturnType = drake::multibody::mpm::internal::SimdScalar<T>;
};

// Computing "float [op] SimdScalar" yields an SimdScalar.
template <typename BinOp, typename T>
struct ScalarBinaryOpTraits<
    float, drake::multibody::mpm::internal::SimdScalar<T>, BinOp> {
  using ReturnType = drake::multibody::mpm::internal::SimdScalar<T>;
};

}  // namespace Eigen

#endif  // DRAKE_DOXYGEN_CXX
