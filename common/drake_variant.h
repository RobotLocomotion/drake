#pragma once

// For now, #include <stx/variant.hpp> via drake_optional.h; there is a lot of
// ceremony to include it correctly, which we only want to repeat once.
#include "drake/common/drake_optional.h"

/// @file
/// Provides drake::variant as an alias for the appropriate implementation of
/// std::variant or or stx::variant for the C++ toolchain being used.

namespace drake {

template <typename... Types>
using variant = stx::variant<Types...>;

#ifdef STX_HAVE_STD_VARIANT
// The stx::get alias is missing, so we have to do this instead.
using std::get;
#else
using stx::get;
#endif

using stx::holds_alternative;

// N.B. We don't alias stx::get_if, because it's signature (reference argument)
// differs from what was standardized (pointer argument).

}  // namespace drake
