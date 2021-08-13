/** @file
Utilities for arithmetic on AutoDiffScalar. */

#pragma once

#include <cmath>
#include <tuple>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"

namespace drake {
namespace math {

template <typename Derived>
struct AutoDiffToValueMatrix {
  typedef typename Eigen::Matrix<typename Derived::Scalar::Scalar,
                                 Derived::RowsAtCompileTime,
                                 Derived::ColsAtCompileTime> type;
};

template <typename Derived>
typename AutoDiffToValueMatrix<Derived>::type ExtractValueMatrixFromAutoDiff(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  typename AutoDiffToValueMatrix<Derived>::type value(auto_diff_matrix.rows(),
                                                      auto_diff_matrix.cols());
  for (int i = 0; i < auto_diff_matrix.rows(); i++) {
    for (int j = 0; j < auto_diff_matrix.cols(); ++j) {
      value(i, j) = auto_diff_matrix(i, j).value();
    }
  }
  return value;
}

template <typename Derived>
DRAKE_DEPRECATED("2021-12-01", "Use ExtractValueMatrixFromAutoDiff() instead")
typename AutoDiffToValueMatrix<Derived>::type
    autoDiffToValueMatrix(const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  return ExtractValueMatrixFromAutoDiff(auto_diff_matrix);
}

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

@see DiscardZeroGradient */
template <typename Derived>
typename std::enable_if_t<
    !std::is_same_v<typename Derived::Scalar, double>,
    Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime, 0, Derived::MaxRowsAtCompileTime,
                  Derived::MaxColsAtCompileTime>>
DiscardGradient(const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  return ExtractValueMatrixFromAutoDiff(auto_diff_matrix);
}

/** @see DiscardGradient(). */
template <typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Derived::Scalar, double>,
    const Eigen::MatrixBase<Derived>&>
DiscardGradient(const Eigen::MatrixBase<Derived>& matrix) {
  return matrix;
}

/** @see DiscardGradient(). */
template <typename _Scalar, int _Dim, int _Mode, int _Options>
typename std::enable_if_t<
    !std::is_same_v<_Scalar, double>,
    Eigen::Transform<typename _Scalar::Scalar, _Dim, _Mode, _Options>>
DiscardGradient(const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>&
                    auto_diff_transform) {
  return Eigen::Transform<typename _Scalar::Scalar, _Dim, _Mode, _Options>(
      ExtractValueMatrixFromAutoDiff(auto_diff_transform.matrix()));
}

/** @see DiscardGradient(). */
template <typename _Scalar, int _Dim, int _Mode, int _Options>
typename std::enable_if_t<std::is_same_v<_Scalar, double>,
                          const Eigen::Transform<_Scalar, _Dim, _Mode,
    _Options>&>
DiscardGradient(
    const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& transform) {
  return transform;
}


/** Initializes a single AutoDiff matrix given the corresponding value matrix.

Sets the values of `auto_diff_matrix` (after resizing if necessary) to be equal
to `value_matrix`, and for each element i of `auto_diff_matrix`, resizes the
derivatives vector to `num_derivatives` and sets derivative number
`deriv_num_start` + i to one (all other elements of the derivative vector set
to zero).

@param[in] value_matrix a 'regular' matrix of values
@param[out] auto_diff_matrix AutoDiff matrix set as described above
@param[in] num_derivatives the size of the derivatives vector
    @default the size of `value_matrix`
@param[in] deriv_num_start starting index into derivative vector (i.e. element
    deriv_num_start in derivative vector corresponds to value_matrix(0, 0)).
    @default 0 */
template <typename Derived, typename DerivedAutoDiff>
void InitializeAutoDiffFromValueMatrix(
    const Eigen::MatrixBase<Derived>& value_matrix,
    Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix,
    Eigen::DenseIndex num_derivatives = Eigen::Dynamic,
    Eigen::DenseIndex deriv_num_start = 0) {

  // Any fixed-size dimension of auto_diff_matrix must match the
  // corresponding fixed-size dimension of value_matrix. Any dynamic-size
  // dimension must be dynamic in both matrices.
  static_assert(static_cast<int>(Derived::RowsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::RowsAtCompileTime),
                "auto diff matrix has wrong number of rows at compile time");
  static_assert(static_cast<int>(Derived::ColsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::ColsAtCompileTime),
                "auto diff matrix has wrong number of columns at compile time");

  DRAKE_DEMAND(auto_diff_matrix != nullptr);
  if (num_derivatives == Eigen::Dynamic) num_derivatives = value_matrix.size();

  using ADScalar = typename DerivedAutoDiff::Scalar;
  auto_diff_matrix->resize(value_matrix.rows(), value_matrix.cols());
  Eigen::DenseIndex deriv_num = deriv_num_start;
  for (Eigen::DenseIndex i = 0; i < value_matrix.size(); i++) {
    (*auto_diff_matrix)(i) =
        ADScalar(value_matrix(i), num_derivatives, deriv_num++);
  }
}

template <typename Derived, typename DerivedAutoDiff>
DRAKE_DEPRECATED("2021-12-01",
    "Use InitializeAutoDiffFromValueMatrix() instead")
void initializeAutoDiff(const Eigen::MatrixBase<Derived>& value_matrix,
                        // NOLINTNEXTLINE(runtime/references).
                        Eigen::MatrixBase<DerivedAutoDiff>& auto_diff_matrix,
                        Eigen::DenseIndex num_derivatives = Eigen::Dynamic,
                        Eigen::DenseIndex deriv_num_start = 0) {
  InitializeAutoDiffFromValueMatrix(value_matrix, &auto_diff_matrix,
      num_derivatives, deriv_num_start);
}

/** The appropriate AutoDiffScalar matrix type given the value type and the
number of derivatives at compile time. */
template <typename Derived, int nq>
using AutoDiffMatrixType = Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<typename Derived::Scalar, nq, 1>>,
    Derived::RowsAtCompileTime, Derived::ColsAtCompileTime, 0,
    Derived::MaxRowsAtCompileTime, Derived::MaxColsAtCompileTime>;

/** Initialize a single AutoDiff matrix given the corresponding value matrix.

Create an AutoDiff matrix that matches `value_matrix` in size with derivative
of compile time size `Nq` and runtime size `num_derivatives`.
 Set its values to be equal to `val`, and for each element i of
`auto_diff_matrix`, set derivative number `deriv_num_start` + i to one (all
other derivatives set to zero).

@param[in] value_matrix 'regular' matrix of values
@param[in] num_derivatives the size of the derivatives vector
    @default the size of matrix
@param[in] deriv_num_start starting index into derivative vector (i.e. element
    deriv_num_start in derivative vector corresponds to matrix(0, 0)).
    @default 0
@returns AutoDiff matrix */
template <int Nq = Eigen::Dynamic, typename Derived>
AutoDiffMatrixType<Derived, Nq> InitializeAutoDiffFromValueMatrix(
    const Eigen::MatrixBase<Derived>& value_matrix,
    Eigen::DenseIndex num_derivatives = -1,
    Eigen::DenseIndex deriv_num_start = 0) {
  if (num_derivatives == -1) num_derivatives = value_matrix.size();

  AutoDiffMatrixType<Derived, Nq> auto_diff_matrix(value_matrix.rows(),
                                                   value_matrix.cols());
  InitializeAutoDiffFromValueMatrix(value_matrix, &auto_diff_matrix,
                                    num_derivatives, deriv_num_start);
  return auto_diff_matrix;
}

template <int Nq = Eigen::Dynamic, typename Derived>
DRAKE_DEPRECATED("2021-12-01",
    "Use InitializeAutoDiffFromValueMatrix() instead")
AutoDiffMatrixType<Derived, Nq> initializeAutoDiff(
    const Eigen::MatrixBase<Derived>& value_matrix,
    Eigen::DenseIndex num_derivatives = -1,
    Eigen::DenseIndex deriv_num_start = 0) {
  return InitializeAutoDiffFromValueMatrix(value_matrix,
                                           num_derivatives,
                                           deriv_num_start);
}

namespace internal {
template <typename Derived, typename Scalar>
struct ResizeDerivativesToMatchScalarImpl {
  static void run(const Scalar&, Eigen::MatrixBase<Derived>*) {}
};

template <typename Derived, typename DerivType>
struct ResizeDerivativesToMatchScalarImpl<Derived,
                                          Eigen::AutoDiffScalar<DerivType>> {
  using Scalar = Eigen::AutoDiffScalar<DerivType>;
  static void run(const Scalar& scalar, Eigen::MatrixBase<Derived>* mat) {
    for (int i = 0; i < mat->size(); i++) {
      auto& derivs = (*mat)(i).derivatives();
      if (derivs.size() == 0) {
        derivs.resize(scalar.derivatives().size());
        derivs.setZero();
      }
    }
  }
};
}  // namespace internal

/** Resize derivatives vector of each element of a matrix to match the size
of the derivatives vector of a given scalar.

If the `matrix` and `scalar` inputs are both AutoDiffScalars, resize the
derivatives vector of each element of `matrix` to match
the number of derivatives of `scalar`. This is useful in functions that
return matrices that do not depend on an AutoDiffScalar
argument (e.g. a function with a constant output), while it is desired that
information about the number of derivatives is preserved.

@param scalar an AutoDiffScalar whose derivative size should be preserved in
    `matrix`.
@param matrix a matrix of AutoDiffScalars whose elements will have their
    empty derivative vectors resized to match that of `scalar`. */
template <typename Derived>
void ResizeDerivativesToMatchScalar(const typename Derived::Scalar& scalar,
                                    Eigen::MatrixBase<Derived>* matrix) {
  internal::ResizeDerivativesToMatchScalarImpl<
      Derived, typename Derived::Scalar>::run(scalar, &matrix);
}

template <typename Derived>
DRAKE_DEPRECATED("2021-12-01", "Use ResizeDerivativesToMatchScalar() instead")
// NOLINTNEXTLINE(runtime/references).
void resizeDerivativesToMatchScalar(Eigen::MatrixBase<Derived>& matrix,
                                    const typename Derived::Scalar& scalar) {
  ResizeDerivativesToMatchScalar(scalar, &matrix);
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

/* Determine the total size at compile time of a number of arguments
based on their SizeAtCompileTime static members. Returns Eigen::Dynamic if
total size can't be determined at compile time. */
template <typename... Args>
constexpr int total_size_at_compile_time() {
  return TotalSizeAtCompileTimeHelper<Args...>::eval();
}

/* Determine the total size at runtime of a number of arguments using
their size() methods (base case). */
constexpr Eigen::DenseIndex total_size_at_run_time() { return 0; }

/* Determine the total size at runtime of a number of arguments using
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
    InitializeAutoDiffFromValueMatrix(value,
                                      &auto_diff,
                                      num_derivatives,
                                      deriv_num_start);
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

/* Given a series of Eigen matrices, create a tuple of corresponding
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
  Eigen::DenseIndex dynamic_num_derivs =
      internal::total_size_at_run_time(args...);
  std::tuple<AutoDiffMatrixType<
      Deriveds, internal::total_size_at_compile_time<Deriveds...>()>...>
  result(
      AutoDiffMatrixType<Deriveds,
                         internal::total_size_at_compile_time<Deriveds...>()>(
          args.rows(), args.cols())...);
  auto values = std::forward_as_tuple(args...);
  internal::InitializeAutoDiffTupleHelper<sizeof...(args)>::run(
      values, dynamic_num_derivs, 0, &result);
  return result;
}

template <typename... Deriveds>
DRAKE_DEPRECATED("2021-12-01", "Use InitializeAutoDiffTuple() instead")
std::tuple<AutoDiffMatrixType<
    Deriveds, internal::total_size_at_compile_time<Deriveds...>()>...>
initializeAutoDiffTuple(const Eigen::MatrixBase<Deriveds>&... args) {
  return InitializeAutoDiffTuple(args...);
}

}  // namespace math
}  // namespace drake
