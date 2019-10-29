#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning "drake/common/drake_variant.h" and drake::variant are deprecated and will be removed on or after 2020-02-01. Please use <variant> and std::variant instead.

/// @file
/// Provides drake::variant as a deprecated alias for std::variant.

#include <variant>

namespace drake {

template <typename... Types>
using variant = std::variant<Types...>;
using std::get;
using std::holds_alternative;

}  // namespace drake
