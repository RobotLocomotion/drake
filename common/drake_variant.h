#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning "drake/common/drake_variant.h" and drake::variant are deprecated and will be removed on or after 2020-02-01. Please use <variant> and std::variant instead.

/// @file
/// Provides drake::variant as a deprecated alias for std::variant.

#include <variant>

#include "drake/common/drake_deprecated.h"

namespace drake {

template <typename... Types>
using variant
    DRAKE_DEPRECATED("2020-02-01", "Please use std::variant instead.")
    = std::variant<Types...>;

// DRAKE_DEPRECATED 2020-02-01 Please use std::get instead.
using std::get;

// DRAKE_DEPRECATED 2020-02-01 Please use std::holds_alternative instead.
using std::holds_alternative;

}  // namespace drake
