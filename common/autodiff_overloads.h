/// @file
/// Overloads for STL mathematical operations on AutoDiffScalar.
///
/// Used via argument-dependent lookup (ADL). These functions appear
/// in the Eigen namespace so that ADL can automatically choose between
/// the STL version and the overloaded version to match the type of the
/// arguments. The proper use would be e.g.
///
/// \code{.cc}
///    void mymethod() {
///       using std::isinf;
///       isinf(myval);
///    }
/// \endcode{}
///
/// @note The if_then_else and cond functions for AutoDiffScalar are in
/// namespace drake because cond is defined in namespace drake in
/// "drake/common/cond.h" file.

#pragma once

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
// TODO(soonho-tri): Change to #error.
#warning Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <cmath>
#include <limits>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/dummy_value.h"

namespace drake {

/// Returns the autodiff scalar's value() as a double.  Never throws.
/// Overloads ExtractDoubleOrThrow from common/extract_double.h.
/// @see math::ExtractValue(), math::DiscardGradient()
inline double ExtractDoubleOrThrow(const AutoDiffXd& scalar) {
  return static_cast<double>(scalar.value());
}

/// Returns @p matrix as an Eigen::Matrix<double, ...> with the same size
/// allocation as @p matrix.  Calls ExtractDoubleOrThrow on each element of the
/// matrix, and therefore throws if any one of the extractions fail.
/// @see math::ExtractValue(), math::DiscardGradient()
template <int RowsAtCompileTime, int ColsAtCompileTime,
          int Options, int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
auto ExtractDoubleOrThrow(
    const Eigen::MatrixBase<Eigen::Matrix<
        AutoDiffXd, RowsAtCompileTime, ColsAtCompileTime,
        Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>>& matrix) {
  return matrix
      .unaryExpr([](const AutoDiffXd& value) {
        return ExtractDoubleOrThrow(value);
      })
      .eval();
}

/// Specializes common/dummy_value.h.
template <>
struct dummy_value<AutoDiffXd> {
  static AutoDiffXd get() {
    return std::numeric_limits<double>::quiet_NaN();
  }
};

/// Provides if-then-else expression for Eigen::AutoDiffScalar type. To support
/// Eigen's generic expressions, we use casting to the plain object after
/// applying Eigen::internal::remove_all. It is based on the Eigen's
/// implementation of min/max function for AutoDiffScalar type
/// (https://bitbucket.org/eigen/eigen/src/10a1de58614569c9250df88bdfc6402024687bc6/unsupported/Eigen/src/AutoDiff/AutoDiffScalar.h?at=default&fileviewer=file-view-default#AutoDiffScalar.h-546).
inline
AutoDiffXd if_then_else(bool f_cond, const AutoDiffXd& x, const AutoDiffXd& y) {
  return f_cond ? x : y;
}

/// Provides special case of cond expression for Eigen::AutoDiffScalar type.
template <typename... Rest>
AutoDiffXd cond(bool f_cond, const AutoDiffXd& e_then, Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}

}  // namespace drake
