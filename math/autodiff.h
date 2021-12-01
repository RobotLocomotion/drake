/** @file
Utilities for arithmetic on AutoDiffScalar. */

#pragma once

#include <cmath>
#include <optional>
#include <tuple>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"

namespace drake {
namespace math {


/** Extracts the `value()` portion from a matrix of AutoDiffScalar entries.
(Each entry contains a value and some derivatives.)

@param auto_diff_matrix An object whose Eigen type represents a matrix of
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
Eigen::Matrix<typename Derived::Scalar::Scalar,
              Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
ExtractValue(const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  Eigen::Matrix<typename Derived::Scalar::Scalar,
                Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
      value(auto_diff_matrix.rows(), auto_diff_matrix.cols());

  for (int i = 0; i < auto_diff_matrix.rows(); ++i) {
    for (int j = 0; j < auto_diff_matrix.cols(); ++j) {
      value(i, j) = auto_diff_matrix(i, j).value();
    }
  }
  return value;
}

template <typename Derived>
struct DRAKE_DEPRECATED("2022-02-01", "Used only in deprecated functions.")
    AutoDiffToValueMatrix {
  typedef typename Eigen::Matrix<typename Derived::Scalar::Scalar,
                                 Derived::RowsAtCompileTime,
                                 Derived::ColsAtCompileTime>
      type;
};

// This deprecated function uses the above deprecated struct.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename Derived>
DRAKE_DEPRECATED("2022-02-01", "Use ExtractValue() instead")
typename AutoDiffToValueMatrix<Derived>::type
    autoDiffToValueMatrix(const Eigen::MatrixBase<Derived>& autodiff_matrix) {
  return ExtractValue(autodiff_matrix);
}
#pragma GCC diagnostic pop

/** `B = DiscardGradient(A)` enables casting from a matrix of AutoDiffScalars
to AutoDiffScalar::Scalar type, explicitly throwing away any gradient
information. For a matrix of type, e.g. `MatrixX<AutoDiffXd> A`, the
comparable operation
  `B = A.cast<double>()`
should (and does) fail to compile.  Use `DiscardGradient(A)` if you want to
force the cast (and explicitly declare that information is lost).

This method is overloaded to permit the user to call it for double types and
AutoDiffScalar types (to avoid the calling function having to handle the
two cases differently).

See ExtractValue() for a note on similar Drake functions.

@see ExtractValue(), DiscardZeroGradient() */
template <typename Derived>
typename std::enable_if_t<
    !std::is_same_v<typename Derived::Scalar, double>,
    Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime, 0, Derived::MaxRowsAtCompileTime,
                  Derived::MaxColsAtCompileTime>>
DiscardGradient(const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  return ExtractValue(auto_diff_matrix);
}

/** @see DiscardGradient(). */
template <typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Derived::Scalar, double>,
    const Eigen::MatrixBase<Derived>&>
DiscardGradient(const Eigen::MatrixBase<Derived>& matrix) {
  return matrix;
}

/** Initializes a single AutoDiff matrix given the corresponding value matrix.

Sets the values of `auto_diff_matrix` (after resizing if necessary) to be equal
to `value`, and for each element i of `auto_diff_matrix`, resizes the
derivatives vector to `num_derivatives` and sets derivative number
`deriv_num_start` + i to one (all other elements of the derivative vector set
to zero).

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
void InitializeAutoDiff(
    const Eigen::MatrixBase<Derived>& value,
    Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix) {
  InitializeAutoDiff(value, {}, {}, auto_diff_matrix);
}

template <typename Derived, typename DerivedAutoDiff>
DRAKE_DEPRECATED("2022-02-01", "Use InitializeAutoDiff() instead")
void initializeAutoDiff(const Eigen::MatrixBase<Derived>& value,
                        // NOLINTNEXTLINE(runtime/references).
                        Eigen::MatrixBase<DerivedAutoDiff>& auto_diff_matrix,
                        Eigen::DenseIndex num_derivatives = Eigen::Dynamic,
                        Eigen::DenseIndex deriv_num_start = 0) {
  InitializeAutoDiff(value,
                     num_derivatives == Eigen::Dynamic
                         ? std::nullopt
                         : std::optional<int>(num_derivatives),
                     static_cast<int>(deriv_num_start), &auto_diff_matrix);
}

/** The appropriate AutoDiffScalar matrix type given the value type and the
number of derivatives at compile time. */
template <typename Derived, int nq>
using AutoDiffMatrixType = Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<typename Derived::Scalar, nq, 1>>,
    Derived::RowsAtCompileTime, Derived::ColsAtCompileTime, 0,
    Derived::MaxRowsAtCompileTime, Derived::MaxColsAtCompileTime>;

/** Initializes a single AutoDiff matrix given the corresponding value matrix.

Creates an AutoDiff matrix that matches `value` in size with derivative
of compile time size `nq` and runtime size `num_derivatives`. Sets its values to
be equal to `value`, and for each element i of `auto_diff_matrix`, sets
derivative number `deriv_num_start` + i to one (all other derivatives set to
zero).

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

template <int nq = Eigen::Dynamic, typename Derived>
DRAKE_DEPRECATED("2022-02-01", "Use InitializeAutoDiff() instead")
AutoDiffMatrixType<Derived, nq> initializeAutoDiff(
    const Eigen::MatrixBase<Derived>& mat,
    Eigen::DenseIndex num_derivatives = -1,
    Eigen::DenseIndex deriv_num_start = 0) {
  return InitializeAutoDiff<nq, Derived>(
      mat,
      num_derivatives == -1 ? std::nullopt
                            : std::optional<int>(num_derivatives),
      static_cast<int>(deriv_num_start));
}

namespace internal {
/* Helper for total_size_at_compile_time() (recursive). */
template <typename Head, typename... Tail>
struct TotalSizeAtCompileTimeHelper {
  static constexpr int eval() {
    return Head::SizeAtCompileTime == Eigen::Dynamic ||
                   TotalSizeAtCompileTimeHelper<Tail...>::eval() ==
                       Eigen::Dynamic
               ? Eigen::Dynamic
               : Head::SizeAtCompileTime +
                     TotalSizeAtCompileTimeHelper<Tail...>::eval();
  }
};

/* Helper for total_size_at_compile_time() (base case). */
template <typename Head>
struct TotalSizeAtCompileTimeHelper<Head> {
  static constexpr int eval() { return Head::SizeAtCompileTime; }
};

/* Determines the total size at compile time of a number of arguments
based on their SizeAtCompileTime static members. Returns Eigen::Dynamic if
total size can't be determined at compile time. */
template <typename... Args>
constexpr int total_size_at_compile_time() {
  return TotalSizeAtCompileTimeHelper<Args...>::eval();
}

/* Determines the total size at runtime of a number of arguments using
their size() methods (base case). */
constexpr Eigen::DenseIndex total_size_at_run_time() { return 0; }

/* Determines the total size at runtime of a number of arguments using
their size() methods (recursive). */
template <typename Head, typename... Tail>
Eigen::DenseIndex total_size_at_run_time(const Eigen::MatrixBase<Head>& head,
                                         const Tail&... tail) {
  return head.size() + total_size_at_run_time(tail...);
}

/* Helper for InitializeAutoDiffTuple() (recursive). */
template <size_t index>
struct InitializeAutoDiffTupleHelper {
  template <typename... ValueTypes, typename... AutoDiffTypes>
  static void run(const std::tuple<ValueTypes...>& values,
                  Eigen::DenseIndex num_derivatives,
                  Eigen::DenseIndex deriv_num_start,
                  std::tuple<AutoDiffTypes...>* auto_diffs) {
    DRAKE_DEMAND(auto_diffs != nullptr);
    constexpr size_t tuple_index = sizeof...(AutoDiffTypes) - index;
    const auto& value = std::get<tuple_index>(values);
    auto& auto_diff = std::get<tuple_index>(*auto_diffs);
    auto_diff.resize(value.rows(), value.cols());
    InitializeAutoDiff(value, num_derivatives, deriv_num_start, &auto_diff);
    InitializeAutoDiffTupleHelper<index - 1>::run(
        values, num_derivatives, deriv_num_start + value.size(), auto_diffs);
  }
};

/* Helper for InitializeAutoDiffTuple() (base case). */
template <>
struct InitializeAutoDiffTupleHelper<0> {
  template <typename... ValueTypes, typename... AutoDiffTypes>
  static void run(const std::tuple<ValueTypes...>& values,
                  Eigen::DenseIndex num_derivatives,
                  Eigen::DenseIndex deriv_num_start,
                  const std::tuple<AutoDiffTypes...>* auto_diffs) {
    unused(values, num_derivatives, deriv_num_start, auto_diffs);
  }
};
}  // namespace internal

/* Given a series of Eigen matrices, creates a tuple of corresponding
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
subsequent elements of the first input argument (traversed first by row, then by
column), and so on for subsequent arguments.

@param args a series of Eigen matrices
@returns a tuple of properly initialized AutoDiff matrices corresponding to
    `args` */
template <typename... Deriveds>
std::tuple<AutoDiffMatrixType<
    Deriveds, internal::total_size_at_compile_time<Deriveds...>()>...>
InitializeAutoDiffTuple(const Eigen::MatrixBase<Deriveds>&... args) {
  constexpr int compile_num_derivs =
      internal::total_size_at_compile_time<Deriveds...>();
  const Eigen::DenseIndex dynamic_num_derivs =
      internal::total_size_at_run_time(args...);

  std::tuple<AutoDiffMatrixType<Deriveds, compile_num_derivs>...> result(
      AutoDiffMatrixType<Deriveds, compile_num_derivs>(args.rows(),
                                                       args.cols())...);
  auto values = std::forward_as_tuple(args...);
  internal::InitializeAutoDiffTupleHelper<sizeof...(args)>::run(
      values, dynamic_num_derivs, 0, &result);
  return result;
}

template <typename... Deriveds>
DRAKE_DEPRECATED("2022-02-01", "Use InitializeAutoDiffTuple() instead")
std::tuple<AutoDiffMatrixType<
    Deriveds, internal::total_size_at_compile_time<Deriveds...>()>...>
initializeAutoDiffTuple(const Eigen::MatrixBase<Deriveds>&... args) {
  return InitializeAutoDiffTuple(args...);
}

}  // namespace math
}  // namespace drake
