/** @file
Utilities that relate simultaneously to both autodiff matrices and
gradient matrices. */

#pragma once

#include <algorithm>
#include <optional>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/math/gradient.h"

namespace drake {
namespace math {

/** Extracts the `derivatives()` portion from an AutoDiffScalar matrix into a
pre-existing matrix (resizing if necessary).

@param[in] auto_diff_matrix An object whose Eigen type represents a matrix of
    AutoDiffScalar entries.
@param[in] num_derivatives The number of derivatives to return in case
    the input matrix has none, which we interpret as `num_derivatives` zeroes.
    If `num_derivatives` is supplied and the input matrix has derivatives, the
    sizes must match.
@param[out] gradient An Eigen::Matrix resized if necessary to have rows equal to
    the total size (rows x cols) of the input matrix and number of columns equal
    to the number of derivatives. Each output row corresponds to one entry of
    the input matrix, using the input matrix storage order. For example, in the
    typical case of a ColMajor `auto_diff_matrix`, we have
    `auto_diff_matrix(r, c).derivatives() ==
    gradient_matrix.row(r + c * auto_diff_matrix.rows())`.

@tparam Derived An Eigen type representing a matrix with AutoDiffScalar
    entries. The type will be inferred from the type of the `auto_diff_matrix`
    parameter at the call site.

@pre `gradient != nullptr`.
@throws std::exception if the input matrix has elements with inconsistent,
    non-zero numbers of derivatives.
@throws std::exception if `num_derivatives` is specified but the input matrix
    has a different, non-zero number of derivatives.
@exclude_from_pydrake_mkdoc{This overload is not bound.} */
template <typename Derived>
void ExtractGradient(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix,
    std::optional<int> num_derivatives,
    Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::SizeAtCompileTime,
                  Eigen::Dynamic>* gradient) {
  DRAKE_THROW_UNLESS(gradient != nullptr);
  // Entries in an AutoDiff matrix must all have the same number of derivatives,
  // or 0-length derivatives in which case they are interpreted as all-zero.
  int num_derivatives_from_matrix = 0;
  for (int i = 0; i < auto_diff_matrix.size(); ++i) {
    const int entry_num_derivs =
        static_cast<int>(auto_diff_matrix(i).derivatives().size());
    if (entry_num_derivs == 0) continue;  // Always OK.
    if (num_derivatives_from_matrix != 0 &&
        entry_num_derivs != num_derivatives_from_matrix) {
      throw std::logic_error(fmt::format(
          "ExtractGradient(): Input matrix has elements with inconsistent,"
          " non-zero numbers of derivatives ({} and {}).",
          num_derivatives_from_matrix, entry_num_derivs));
    }
    num_derivatives_from_matrix = entry_num_derivs;
  }

  if (!num_derivatives.has_value()) {
    num_derivatives = num_derivatives_from_matrix;
  } else if (num_derivatives_from_matrix != 0 &&
             num_derivatives_from_matrix != *num_derivatives) {
    throw std::logic_error(fmt::format(
        "ExtractGradient(): Input matrix has {} derivatives, but"
        " num_derivatives was specified as {}. Either the input matrix should"
        " have zero derivatives, or the number should match num_derivatives.",
        num_derivatives_from_matrix, *num_derivatives));
  }

  gradient->resize(auto_diff_matrix.size(), *num_derivatives);
  if (gradient->size() == 0) {
    return;
  }
  for (int i = 0; i < auto_diff_matrix.size(); ++i) {
    auto gradient_row = gradient->row(i).transpose();
    if (auto_diff_matrix(i).derivatives().size() == 0) {
      gradient_row.setZero();
    } else {
      gradient_row = auto_diff_matrix(i).derivatives();
    }
  }
}

/** Returns the `derivatives()` portion from a matrix of AutoDiffScalar
entries. (Each entry contains a value and derivatives.)

@param[in] auto_diff_matrix An object whose Eigen type represents a matrix of
    AutoDiffScalar entries.
@param[in] num_derivatives (Optional) The number of derivatives to return in
    case the input matrix has none, which we interpret as `num_derivatives`
    zeroes. If `num_derivatives` is supplied and the input matrix has
    derivatives, the sizes must match.
@retval gradient_matrix An Eigen::Matrix with number of rows equal to the
    total size (rows x cols) of the input matrix and number of columns equal
    to the number of derivatives. Each output row corresponds to one entry of
    the input matrix, using the input matrix storage order. For example, in
    the typical case of a ColMajor `auto_diff_matrix`, we have
    `auto_diff_matrix(r, c).derivatives() ==
    gradient_matrix.row(r + c * auto_diff_matrix.rows())`.

@tparam Derived An Eigen type representing a matrix with AutoDiffScalar
    entries. The type will be inferred from the type of the `auto_diff_matrix`
    parameter at the call site.

@throws std::exception if the input matrix has elements with inconsistent,
    non-zero numbers of derivatives.
@throws std::exception if `num_derivatives` is specified but the input matrix
    has a different, non-zero number of derivatives.*/
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::SizeAtCompileTime,
              Eigen::Dynamic>
ExtractGradient(const Eigen::MatrixBase<Derived>& auto_diff_matrix,
                std::optional<int> num_derivatives = {}) {
  Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::SizeAtCompileTime,
                Eigen::Dynamic>
      gradient;
  ExtractGradient(auto_diff_matrix, num_derivatives, &gradient);
  return gradient;
}

/** Initializes an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@param[out] auto_diff_matrix The matrix of AutoDiffScalars. Will be resized as
    needed to have the same dimensions as the value matrix. Must have the same
    storage order as `value`.
@exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
template <typename DerivedValue, typename DerivedGradient,
          typename DerivedAutoDiff>
void InitializeAutoDiff(const Eigen::MatrixBase<DerivedValue>& value,
                        const Eigen::MatrixBase<DerivedGradient>& gradient,
                        Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix) {
  // Any fixed-size dimensions of the parameters must be consistent in
  // corresponding dimensions. Any dynamic-size dimensions must be dynamic
  // in any corresponding dimensions.
  static_assert(static_cast<int>(DerivedValue::SizeAtCompileTime) ==
                    static_cast<int>(DerivedGradient::RowsAtCompileTime),
                "gradient has wrong number of rows at compile time");
  using ExpectedAutoDiffType =
      AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>;
  static_assert(static_cast<int>(ExpectedAutoDiffType::RowsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::RowsAtCompileTime),
                "auto diff matrix has wrong number of rows at compile time");
  static_assert(static_cast<int>(ExpectedAutoDiffType::ColsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::ColsAtCompileTime),
                "auto diff matrix has wrong number of columns at compile time");
  static_assert(static_cast<int>(DerivedAutoDiff::IsRowMajor) ==
                    static_cast<int>(DerivedValue::IsRowMajor),
                "auto diff matrix has wrong storage order at compile time");

  // Verify that the Scalar types of the Value matrix and AutoDiff result
  // are the same.
  static_assert(std::is_same_v<typename DerivedAutoDiff::Scalar,
                               typename ExpectedAutoDiffType::Scalar>,
                "wrong auto diff scalar type");

  DRAKE_DEMAND(auto_diff_matrix != nullptr);
  DRAKE_DEMAND(value.size() == gradient.rows() &&
               "gradient has wrong number of rows at runtime");

  // Can't resize() a MatrixBase -- downcast to the actual type.
  DerivedAutoDiff& auto_diff = auto_diff_matrix->derived();
  auto_diff.resize(value.rows(), value.cols());
  for (Eigen::Index row = 0; row < auto_diff.size(); ++row) {
    auto_diff(row) = {value(row), gradient.row(row).transpose()};
  }
}

/** Returns an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@retval auto_diff_matrix The matrix of AutoDiffScalars. Will have the same
    dimensions and storage order as the value matrix.
@pydrake_mkdoc_identifier{value_and_gradient} */
template <typename DerivedValue, typename DerivedGradient>
AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
InitializeAutoDiff(const Eigen::MatrixBase<DerivedValue>& value,
                   const Eigen::MatrixBase<DerivedGradient>& gradient) {
  AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
      auto_diff_matrix(value.rows(), value.cols());
  InitializeAutoDiff(value, gradient, &auto_diff_matrix);
  return auto_diff_matrix;
}

/** `B = DiscardZeroGradient(A, precision)` enables casting from a matrix of
AutoDiffScalars to AutoDiffScalar::Scalar type, but first checking that
the gradient matrix is empty or zero.  For a matrix of type, e.g.
`MatrixX<AutoDiffXd> A`, the comparable operation
  `B = A.cast<double>()`
should (and does) fail to compile.  Use `DiscardZeroGradient(A)` if you want
to force the cast (and the check).

When called with a matrix that is already of type `double`, this function
returns a _reference_ to the argument without any copying. This efficiently
avoids extra copying, but be careful about reference lifetimes!

See ExtractValue() for a note on similar Drake functions.

@param precision is passed to Eigen's isZero(precision) to evaluate whether
the gradients are zero.
@throws std::exception if the gradients were not empty nor zero.
@see DiscardGradient() */
template <typename Derived>
decltype(auto) DiscardZeroGradient(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix,
    double precision = Eigen::NumTraits<double>::dummy_precision()) {
  if constexpr (std::is_same_v<typename Derived::Scalar, double>) {
    unused(precision);
    return auto_diff_matrix;
  } else {
    const auto gradients = ExtractGradient(auto_diff_matrix);
    if (gradients.size() == 0 || gradients.isZero(precision)) {
      return ExtractValue(auto_diff_matrix);
    }
    throw std::runtime_error(
        "Casting AutoDiff to value but gradients are not zero.");
  }
}

/**
 * Given a matrix of AutoDiffScalars, returns the size of the
 * derivatives.
 * @throw runtime_error if some entry has different (non-zero) number of
 * derivatives as the others.
 */
template <typename Derived>
typename std::enable_if<!std::is_same_v<typename Derived::Scalar, double>,
                        int>::type
GetDerivativeSize(const Eigen::MatrixBase<Derived>& A) {
  int num_derivs = 0;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      if (A(i, j).derivatives().size() != 0) {
        if (num_derivs != 0 && A(i, j).derivatives().size() != num_derivs) {
          throw std::runtime_error(fmt::format(
              "GetDerivativeSize(): A({}, {}).derivatives() has size "
              "{}, while another entry has size {}",
              i, j, A(i, j).derivatives().size(), num_derivs));
        }
        num_derivs = A(i, j).derivatives().size();
      }
    }
  }
  return num_derivs;
}

/**
 * Determines if a and b are equal. a equals to b if they have the same value
 * and gradients.
 * TODO(hongkai.dai) implement and use std::equal_to<> for comparing Eigen
 * vector of AutoDiffXd.
 **/
bool AreAutoDiffVecXdEqual(const Eigen::Ref<const VectorX<AutoDiffXd>>& a,
                           const Eigen::Ref<const VectorX<AutoDiffXd>>& b);

}  // namespace math
}  // namespace drake
