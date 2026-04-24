#pragma once

#include <limits>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

/* This file contains Eigen-related specializations for Drake's AutoDiff
scalar type (plus the intimately related std::numeric_limits specialization).

The specializations both add basic capability (e.g., NumTraits) as well as
improve performance (e.g., fewer copies that Eigen would naively use).

NOTE: This file should never be included directly, rather only from
auto_diff.h in a very specific order. */

#ifndef DRAKE_DOXYGEN_CXX

namespace std {
template <>
class numeric_limits<drake::ad::AutoDiff> : public numeric_limits<double> {};
}  // namespace std

namespace Eigen {

// === See Eigen/src/Core/NumTraits.h ===

// See https://eigen.tuxfamily.org/dox/structEigen_1_1NumTraits.html.
// We'll Inherit the constants from `double`, but be sure to fatten a few types
// up to full AutoDiff where necessary.
template <>
struct NumTraits<drake::ad::AutoDiff> : public NumTraits<double> {
  // This refers to the "real part" of a complex number (e.g., std::complex).
  // Because we're not a complex number, it's just the same type as ourselves.
  using Real = drake::ad::AutoDiff;

  // This promotes integer types during operations like quotients, square roots,
  // etc. We're already floating-point, so it's just the same type as ourselves.
  using NonInteger = drake::ad::AutoDiff;

  // Eigen says "If you don't know what this means, just use [your type] here."
  using Nested = drake::ad::AutoDiff;

  // Our constructor is required during matrix storage initialization.
  enum { RequireInitialization = 1 };
};

// Computing "ADS [op] double" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<drake::ad::AutoDiff, double, BinOp> {
  using ReturnType = drake::ad::AutoDiff;
};

// Computing "double [op] ADS" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<double, drake::ad::AutoDiff, BinOp> {
  using ReturnType = drake::ad::AutoDiff;
};

}  // namespace Eigen

#endif  // DRAKE_DOXYGEN_CXX

namespace drake {
namespace ad {
namespace internal {

/* Eigen promises "automatic conversion of the inner product to a scalar", so
when we calculate an inner product we need to return this magic type that like
acts both a Matrix1<AutoDiff> and a scalar AutoDiff. There does not appear to be
any practical way to mimic what Eigen does, other than multiple inheritance. */
class DegenerateAutoDiffInnerProduct
    : public AutoDiff,
      public Eigen::Map<Eigen::Matrix<AutoDiff, 1, 1>> {
 public:
  DegenerateAutoDiffInnerProduct() : Map(this) {}
  void resize(int rows, int cols) { DRAKE_ASSERT(rows == 1 && cols == 1); }
};

/* Helper to look up the return type we'll use for an AutoDiff matmul. */
template <typename MatrixL, typename MatrixR>
using AutoDiffMatMulResult =
    std::conditional_t<MatrixL::RowsAtCompileTime == 1 &&
                           MatrixR::ColsAtCompileTime == 1,
                       DegenerateAutoDiffInnerProduct,
                       Eigen::Matrix<AutoDiff, MatrixL::RowsAtCompileTime,
                                     MatrixR::ColsAtCompileTime>>;

/* Optimized implementations of BLAS GEMM for autodiff types to take advantage
of scalar type specializations. With our current mechanism for hooking this into
Eigen, we only need to support the simplified form C ⇐ A@B rather than the more
general C ⇐ αA@B+βC of typical GEMM; if we figure out how to hook into Eigen's
expression templates, we could expand to the more general form. We group these
functions using a struct so that the friendship declaration with ad::AutoDiff
and/or internal::Partials would be straightforward in case we ever need it.
@tparam reverse When true, calculates B@A instead of A@B. */
template <bool reverse>
struct Gemm {
  Gemm() = delete;
  // Allow for passing numpy.ndarray without copies. (This isn't yet bound in
  // pydrake, but might be in the future.)
  template <typename T>
  using MatrixRef = Eigen::Ref<const MatrixX<T>, 0, StrideX>;
  // Matrix product for AutoDiff, AutoDiff.
  // When reverse == false, sets result to A * B.
  // When reverse == true, sets result to B * A.
  static void CalcAA(const MatrixRef<AutoDiff>& A, const MatrixRef<AutoDiff>& B,
                     EigenPtr<MatrixX<AutoDiff>> result);
};

}  // namespace internal

// Matrix<AutoDiff> * Matrix<AutoDiff> => Matrix<AutoDiff>
template <typename MatrixL, typename MatrixR>
internal::AutoDiffMatMulResult<MatrixL, MatrixR> operator*(const MatrixL& lhs,
                                                           const MatrixR& rhs)
  requires std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
           std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
           std::is_same_v<typename MatrixL::Scalar, AutoDiff> &&
           std::is_same_v<typename MatrixR::Scalar, AutoDiff>
{
  internal::AutoDiffMatMulResult<MatrixL, MatrixR> result;
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = false;
  internal::Gemm<reverse>::CalcAA(lhs, rhs, &result);
  return result;
}

}  // namespace ad
}  // namespace drake
