#pragma once

#include <limits>

namespace drake {

/// Provides a "dummy" value for a ScalarType -- a value that is unlikely to be
/// mistaken for a purposefully-computed value, useful for initializing a value
/// before the true result is available.
///
/// Defaults to using std::numeric_limits::quiet_NaN when available; it is a
/// compile-time error to call the unspecialized dummy_value::get() when
/// quiet_NaN is unavailable.
///
/// See autodiff_overloads.h to use this with Eigen's AutoDiffScalar.
template <typename T>
struct dummy_value {
  static constexpr T get() {
    static_assert(std::numeric_limits<T>::has_quiet_NaN,
                  "Custom scalar types should specialize this struct");
    return std::numeric_limits<T>::quiet_NaN();
  }
};

template <>
struct dummy_value<int> {
  static constexpr int get() {
    // D is for "Dummy".  We assume as least 32 bits (per cppguide) -- if `int`
    // is larger than 32 bits, this will leave some fraction of the bytes zero
    // instead of 0xDD, but that's okay.
    return 0xDDDDDDDD;
  }
};

}  // namespace drake
