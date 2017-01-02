#pragma once

#include <stdexcept>

#include "drake/common/nice_type_name.h"
#include "drake/common/number_traits.h"

namespace drake {

/// Converts a ScalarType value to a double, failing at runtime (not compile
/// time) if the type cannot be converted to a double.
///
/// This function is useful for writing ScalarType-generic code that (1) is
/// only intended to be used on numeric types, (2) can reasonably discard any
/// supplemental scalar data, e.g., the derivatives of an AutoDiffScalar, and
/// (3) is reasonable to fail at runtime if called with a non-numeric
/// ScalarType.
///
/// The default implementation throws an exception.  ScalarTypes that are
/// numeric must overload this method to provide an appropriate extraction.
/// An overload for `double` is already provided.
///
/// See autodiff_overloads.h to use this with Eigen's AutoDiffScalar.
/// See number_traits.h for specifying which ScalarTypes are numeric.
template <typename T>
double ExtractDoubleOrThrow(const T& scalar) {
  static_assert(!is_numeric<T>::value,
                "Numeric scalar types should overload this function");
  throw std::runtime_error(NiceTypeName::Get<T>() +
                           " cannot be converted to a double");
}

/// Returns the @p scalar a double.  Never throws.
inline double ExtractDoubleOrThrow(double scalar) { return scalar; }

}  // namespace drake
