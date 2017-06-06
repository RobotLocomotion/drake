#pragma once

/// @file
/// Provides drake::optional as an alias for the appropriate implementation of
/// std::optional or std::experimental::optional or boost::optional on the
/// given platform.

// See if the C++ standard library already provides optional in some way.
#if defined(DRAKE_DOXYGEN_CXX) ||              \
    defined(__GNUG__) && (                     \
        defined(__clang__) || ((               \
            __GNUC__ * 10000                   \
            + __GNUC_MINOR__ * 100             \
            + __GNUC_PATCHLEVEL__) >= 40900))

#include <experimental/optional>

#include "drake/common/drake_compat.h"

namespace drake {

template <typename T>
using optional = std::experimental::optional<T>;

constexpr auto nullopt = std::experimental::nullopt;

}  // namespace drake

#else  // The C++ standard library doesn't provide optional.

#include <utility>

#include <boost/optional.hpp>

#include "drake/common/drake_compat.h"

namespace drake {

template <typename T>
class optional : public boost::optional<T> {
 public:
  using boost::optional<T>::optional;
  template <typename U>
  T value_or(U&& default_value) const {
    return static_cast<bool>(*this) ?
        **this :
        static_cast<T>(std::forward<U>(default_value));
  }
};

constexpr auto nullopt = boost::none;

}  // namespace drake

#endif  // C++ version check
