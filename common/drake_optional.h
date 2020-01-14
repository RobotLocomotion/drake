#pragma once

// NOLINTNEXTLINE(whitespace/line_length)
#warning "drake/common/drake_optional.h" and drake::optional are deprecated and will be removed on or after 2020-02-01. Please use <optional> and std::optional instead.

/// @file
/// Provides drake::optional as a deprecated alias for std::optional.

#include <optional>

#include "drake/common/drake_deprecated.h"

namespace drake {

template <typename T>
using optional
    DRAKE_DEPRECATED("2020-02-01", "Please use std::optional instead.")
    = std::optional<T>;

DRAKE_DEPRECATED("2020-02-01", "Please use std::nullopt instead.")
constexpr auto nullopt = std::nullopt;

using nullopt_t
    DRAKE_DEPRECATED("2020-02-01", "Please use std::nullopt_t instead.")
    = std::nullopt_t;

}  // namespace drake
