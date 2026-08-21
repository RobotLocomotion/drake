/** @file
Utilities for arithmetic on AutoDiffScalar. */

#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <tuple>
#include <utility>

#include <Eigen/Dense>

#if DRAKE_INTERNAL_USE_EIGEN_LEGACY_AUTODIFF == 1
#include <unsupported/Eigen/AutoDiff>
#endif

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace math {

/** Extracts the `value()` portion from an AutoDiffScalar matrix into a
pre-existing matrix (resizing if necessary).

@param[in] auto_diff_matrix An object whose Eigen type represents a matrix of
  AutoDiffScalar entries.
@param[out] value An Eigen::Matrix resized if necessary to the same size as
  the input matrix, and copies only the value portion of each entry, without
  the derivatives.
@tparam Derived An Eigen type representing a matrix with AutoDiffScalar
  entries. The type will be inferred from the type of the `auto_diff_matrix`
  parameter at the call site.
@pre `value != nullptr`.
@exclude_from_pydrake_mkdoc{This overload is not bound.} */
template <typename Derived>
void ExtractValue(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix,
    MatrixLikewise<typename Derived::Scalar::Scalar, Derived>* value) {
  DRAKE_THROW_UNLESS(value != nullptr);
  value->resize(auto_diff_matrix.rows(), auto_diff_matrix.cols());
  for (int i = 0; i < auto_diff_matrix.rows(); ++i) {
    for (int j = 0; j < auto_diff_matrix.cols(); ++j) {
      (*value)(i, j) = auto_diff_matrix(i, j).value();
    }
  }
}

/** Returns the `value()` portion from a matrix of AutoDiffScalar entries.
(Each entry contains a value and some derivatives.)

@param[in] auto_diff_matrix An object whose Eigen type represents a matrix of
    AutoDiffScalar entries.
@retval value An Eigen::Matrix of the same dimensions as the input
    matrix, but containing only the value portion of each entry, without the
    derivatives.
@tparam Derived An Eigen type representing a matrix with AutoDiffScalar
    entries. The type will be inferred from the type of the `auto_diff_matrix`
    parameter at the call site.

<!-- Don't remove this note without fixing cross references to it in
     Discard[Zero]Gradient(). -->
@note Drake provides several similar functions: DiscardGradient() is specialized
so that it can be applied to a `Matrix<T>` where T could be an AutoDiffScalar
or an ordinary double, in which case it returns the original matrix at no cost.
DiscardZeroGradient() is similar but requires that the discarded gradient was
zero. drake::ExtractDoubleOrThrow() has many specializations, including one for
`Matrix<AutoDiffScalar>` that behaves identically to ExtractValue().

@see DiscardGradient(), drake::ExtractDoubleOrThrow() */
template <typename Derived>
MatrixLikewise<typename Derived::Scalar::Scalar, Derived> ExtractValue(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  MatrixLikewise<typename Derived::Scalar::Scalar, Derived> value(
      auto_diff_matrix.rows(), auto_diff_matrix.cols());
  ExtractValue(auto_diff_matrix, &value);
  return value;
}

/** `B = DiscardGradient(A)` enables casting from a matrix of AutoDiffScalars
to AutoDiffScalar::Scalar type, explicitly throwing away any gradient
information. For a matrix of type, e.g. `MatrixX<AutoDiffXd> A`, the
comparable operation
  `B = A.cast<double>()`
should (and does) fail to compile.  Use `DiscardGradient(A)` if you want to
force the cast (and explicitly declare that information is lost).

When called with a matrix that is already of type `double`, this function
returns a _reference_ to the argument without any copying. This efficiently
avoids extra copying, but be careful about reference lifetimes!

See ExtractValue() for a note on similar Drake functions.

@see ExtractValue(), DiscardZeroGradient() */
template <typename Derived>
decltype(auto) DiscardGradient(const Eigen::MatrixBase<Derived>& matrix) {
  if constexpr (std::is_same_v<typename Derived::Scalar, double>) {
    return matrix;
  } else {
    return ExtractValue(matrix);
  }
}

/** Initializes a single AutoDiff matrix given the corresponding value matrix.

Sets the values of `auto_diff_matrix` (after resizing if necessary) to be equal
to `value`, and for each element i of `auto_diff_matrix`, resizes the
derivatives vector to `num_derivatives` and sets derivative number
`deriv_num_start` + i to one (all other elements of the derivative vector set
to zero).

When `value` and `auto_diff_matrix` are matrices (rather than just vectors)
note in particular that the derivative numbers count up using the _storage
order_ of `value(i)` and `autodiff_matrix(i)` and so the ColMajor vs RowMajor
storage order of the two must match.

@param[in] value a 'regular' matrix of values
@param[in] num_derivatives (Optional) size of the derivatives vector
    @default total size of the value matrix
@param[in] deriv_num_start (Optional) starting index into derivative vector
    (i.e. element deriv_num_start in derivative vector corresponds to
    value(0, 0)).
    @default 0
@param[out] auto_diff_matrix AutoDiff matrix set as described above

@exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
template <typename Derived, typename DerivedAutoDiff>
void InitializeAutoDiff(const Eigen::MatrixBase<Derived>& value,
                        std::optional<int> num_derivatives,
                        std::optional<int> deriv_num_start,
                        Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix) {
  // Any fixed-size dimension of auto_diff_matrix must match the
  // corresponding fixed-size dimension of value. Any dynamic-size
  // dimension must be dynamic in both matrices.
  static_assert(static_cast<int>(Derived::RowsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::RowsAtCompileTime),
                "auto diff matrix has wrong number of rows at compile time");
  static_assert(static_cast<int>(Derived::ColsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::ColsAtCompileTime),
                "auto diff matrix has wrong number of columns at compile time");
  static_assert(static_cast<int>(Derived::IsRowMajor) ==
                    static_cast<int>(DerivedAutoDiff::IsRowMajor),
                "auto diff matrix has wrong storage order at compile time");

  DRAKE_DEMAND(auto_diff_matrix != nullptr);
  if (!num_derivatives.has_value()) num_derivatives = value.size();

  using ADScalar = typename DerivedAutoDiff::Scalar;
  auto_diff_matrix->resize(value.rows(), value.cols());
  int deriv_num = deriv_num_start.value_or(0);
  for (int i = 0; i < value.size(); ++i) {
    (*auto_diff_matrix)(i) = ADScalar(value(i), *num_derivatives, deriv_num++);
  }
}

/** Alternate signature provides default values for the number of derivatives
(dynamic, determined at run time) and the starting index (0).

@exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
template <typename Derived, typename DerivedAutoDiff>
void InitializeAutoDiff(const Eigen::MatrixBase<Derived>& value,
                        Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix) {
  InitializeAutoDiff(value, {}, {}, auto_diff_matrix);
}

/** The appropriate AutoDiffScalar matrix type given the value type and the
number of derivatives at compile time. */
#if DRAKE_INTERNAL_USE_EIGEN_LEGACY_AUTODIFF == 1
template <typename Derived, int nq>
using AutoDiffMatrixType =
    MatrixLikewise<Eigen::AutoDiffScalar<Vector<typename Derived::Scalar, nq> >,
                   Derived>;
#else
template <typename Derived, int nq>
using AutoDiffMatrixType = MatrixLikewise<AutoDiffXd, Derived>;
#endif

/** Initializes a single AutoDiff matrix given the corresponding value matrix.

Creates an AutoDiff matrix that matches `value` in size with derivative
of compile time size `nq` and runtime size `num_derivatives`. Sets its values to
be equal to `value`, and for each element i of `auto_diff_matrix`, sets
derivative number `deriv_num_start` + i to one (all other derivatives set to
zero).

When `value` is a matrix (rather than just a vector) note in particular that
the return value will use the same storage order (ColMajor vs RowMajor) and
that the derivative numbers count up using the _storage order_ of `value(i)`.

@param[in] value 'regular' matrix of values
@param[in] num_derivatives (Optional) size of the derivatives vector
    @default total size of the value matrix
@param[in] deriv_num_start (Optional) starting index into derivative vector
    (i.e. element deriv_num_start in derivative vector corresponds to
    matrix(0, 0)).
    @default 0
@retval auto_diff_matrix The result as described above.

@pydrake_mkdoc_identifier{just_value} */
template <int nq = Eigen::Dynamic, typename Derived>
AutoDiffMatrixType<Derived, nq> InitializeAutoDiff(
    const Eigen::MatrixBase<Derived>& value,
    std::optional<int> num_derivatives = {},
    std::optional<int> deriv_num_start = {}) {
  AutoDiffMatrixType<Derived, nq> auto_diff_matrix(value.rows(), value.cols());
  InitializeAutoDiff(value, num_derivatives.value_or(value.size()),
                     deriv_num_start.value_or(0), &auto_diff_matrix);
  return auto_diff_matrix;
}

/** Given a series of Eigen matrices, creates a tuple of corresponding
AutoDiff matrices with values equal to the input matrices and properly
initialized derivative vectors.

The size of the derivative vector of each element of the matrices in the
output tuple will be the same, and will equal the sum of the number of
elements of the matrices in `args`. If all of the matrices in `args` have fixed
size, then the derivative vectors will also have fixed size (being the sum of
the sizes at compile time of all of the input arguments), otherwise the
derivative vectors will have dynamic size. The 0th element of the derivative
vectors will correspond to the derivative with respect to the 0th element of the
first argument. Subsequent derivative vector elements correspond first to
subsequent elements of the first input argument (traversed in the same order as
InitializeAutoDiff() for that matrix), and so on for subsequent arguments.

@param args a series of Eigen matrices
@returns a tuple of properly initialized AutoDiff matrices corresponding to
    `args` */
template <typename... Deriveds>
auto InitializeAutoDiffTuple(const Eigen::MatrixBase<Deriveds>&... args) {
  // Compute the total compile-time size of all args (or Dynamic, if unknown).
  // Refer to https://en.cppreference.com/w/cpp/language/fold for the syntax.
  constexpr int nq = ((Deriveds::SizeAtCompileTime != Eigen::Dynamic) && ...)
                         ? (static_cast<int>(Deriveds::SizeAtCompileTime) + ...)
                         : Eigen::Dynamic;

  // Compute each deriv_num_start value and then the total runtime size.
  constexpr size_t N = sizeof...(args);
  const std::array<int, N> sizes{static_cast<int>(args.size())...};
  std::array<int, N + 1> deriv_num_starts = {0};
  for (size_t i = 1; i <= N; ++i) {
    deriv_num_starts[i] = deriv_num_starts[i - 1] + sizes[i - 1];
  }
  const int num_derivatives = deriv_num_starts.back();

  // Allocate the result.
  std::tuple<AutoDiffMatrixType<Deriveds, nq>...> result(
      AutoDiffMatrixType<Deriveds, nq>(args.rows(), args.cols())...);

  // Set the values and gradients of the result using InitializeAutoDiff from
  // each Matrix in 'args...'. This is a "constexpr for" loop for 0 <= I < N.
  auto args_tuple = std::forward_as_tuple(args...);
  [&]<size_t... I>(std::integer_sequence<size_t, I...>&&) {
    (InitializeAutoDiff(std::get<I>(args_tuple), num_derivatives,
                        std::get<I>(deriv_num_starts), &std::get<I>(result)),
     ...);
  }(std::make_index_sequence<N>{});

  return result;
}

}  // namespace math
}  // namespace drake
