#pragma once

#include <stx/variant.hpp>

/// @file
/// Provides drake::variant as an alias for the appropriate implementation of
/// std::variant or or stx::variant for the C++ toolchain being used.

namespace drake {

template <typename... Types>
using variant = stx::variant<Types...>;

using stx::get;
using stx::get_if;
using stx::holds_alternative;

}  // namespace drake
