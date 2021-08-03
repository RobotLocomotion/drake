#pragma once

#include <stdexcept>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"

namespace drake {

/// Converts a ScalarType value to a double, failing at runtime (not compile
/// time) if the type cannot be converted to a double.
///
/// This function is useful for writing ScalarType-generic code that (1) can
/// reasonably discard any supplemental scalar data, e.g., the derivatives of an
/// AutoDiffScalar, and (2) is reasonable to fail at runtime if the extraction
/// fails.
///
/// The default implementation throws an exception.  ScalarTypes that can hold a
/// numeric value must overload this method to provide an appropriate
/// extraction.  An overload for `double` is already provided.
///
/// See autodiff_overloads.h to use this with Eigen's AutoDiffScalar.
/// See symbolic_expression.h to use this with symbolic::Expression.
template <typename T>
DRAKE_DEPRECATED("2020-08-01",
                 "Provide a specific overload of ExtractDoubleOrThrow for any "
                 "type that really is sensible at compile time and should "
                 "defer failure to runtime; this version was too generic.")
typename std::enable_if_t<!is_eigen_type<T>::value, double>
ExtractDoubleOrThrow(const T&) {
  throw std::runtime_error(NiceTypeName::Get<T>() +
                           " cannot be converted to a double");
}

/// Returns @p scalar as a double.  Never throws.
inline double ExtractDoubleOrThrow(double scalar) { return scalar; }

/// Returns @p matrix as an Eigen::Matrix<double, ...> with the same size
/// allocation as @p matrix.  Calls ExtractDoubleOrThrow on each element of the
/// matrix, and therefore throws if any one of the extractions fail.
template <typename Derived>
typename std::enable_if_t<
    std::is_same_v<typename Derived::Scalar, double>,
    Eigen::Matrix<double, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime, Derived::Options,
                  Derived::MaxRowsAtCompileTime, Derived::MaxColsAtCompileTime>>
ExtractDoubleOrThrow(const Eigen::MatrixBase<Derived>& matrix) {
  return matrix.unaryExpr([](const typename Derived::Scalar& value) {
        return ExtractDoubleOrThrow(value);
      })
      .eval();
}

}  // namespace drake
