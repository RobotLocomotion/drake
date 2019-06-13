#pragma once

/// @file
/// Provides drake::variant as an alias for the appropriate implementation of
/// std::variant or stx::variant for the C++ toolchain being used.

// For now, #include <stx/variant.hpp> via drake_optional.h; there is a lot of
// ceremony to include it correctly, which we only want to repeat once.
#include "drake/common/drake_optional.h"

namespace drake {

#ifdef STX_NO_STD_VARIANT
template <typename... Types>
using variant = stx::variant<Types...>;
using stx::get;
using stx::holds_alternative;
#else
template <typename... Types>
using variant = std::variant<Types...>;
using std::get;
using std::holds_alternative;
#endif

// N.B. We don't alias stx::get_if, because it's signature (reference argument)
// differs from what was standardized (pointer argument).

}  // namespace drake
