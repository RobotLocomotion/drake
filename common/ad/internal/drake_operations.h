#pragma once

/* This file contains free function operators for Drake's AutoDiff type, for
functions that are Drake-specific rather than from the standard library.

NOTE: This file should never be included directly, rather only from auto_diff.h
in a very specific order. */

namespace drake {

/** Returns the autodiff scalar's value() as a double. Never throws.
Overloads ExtractDoubleOrThrow() from `drake/common/extract_double.h`.
@see math::ExtractValue(), math::DiscardGradient() */
inline double ExtractDoubleOrThrow(const ad::AutoDiff& scalar) {
  return scalar.value();
}

/** Returns `matrix` as an Eigen::Matrix<double, ...> with the same size
allocation as `matrix`. Calls ExtractDoubleOrThrow on each element of the
matrix, and therefore throws if any one of the extractions fail.
Overloads ExtractDoubleOrThrow() from `drake/common/extract_double.h`.
@see math::ExtractValue(), math::DiscardGradient() */
template <int RowsAtCompileTime, int ColsAtCompileTime, int Options,
          int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
auto ExtractDoubleOrThrow(
    const Eigen::MatrixBase<
        Eigen::Matrix<ad::AutoDiff, RowsAtCompileTime, ColsAtCompileTime,
                      Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>>&
        matrix) {
  return matrix
      .unaryExpr([](const ad::AutoDiff& value) {
        return ExtractDoubleOrThrow(value);
      })
      .eval();
}

/** Provides if-then-else expression for AutoDiff.
Overloads if_then_else() from `drake/common/double_overloads.h`. */
inline ad::AutoDiff if_then_else(bool f_cond, const ad::AutoDiff& x,
                                 const ad::AutoDiff& y) {
  return f_cond ? x : y;
}

/** Provides cond's base case for AutoDiff.
Overloads cond() from `drake/common/double_overloads.h`. */
inline ad::AutoDiff cond(const ad::AutoDiff& e) {
  return e;
}

/** Provides cond base for AutoDiff.
Overloads cond() from `drake/common/double_overloads.h`. */
template <typename... Rest>
inline ad::AutoDiff cond(bool f_cond, const ad::AutoDiff& e_then,
                         Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}

}  // namespace drake
