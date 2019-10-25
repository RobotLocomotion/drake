#pragma once

/// @file
/// Provides drake::variant as an alias for std::variant.

#include <variant>

namespace drake {

template <typename... Types>
using variant = std::variant<Types...>;
using std::get;
using std::holds_alternative;

}  // namespace drake
