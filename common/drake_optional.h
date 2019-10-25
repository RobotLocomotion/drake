#pragma once

/// @file
/// Provides drake::optional as an alias for the std::optional.

#include <optional>

namespace drake {

template <typename T>
using optional = std::optional<T>;
constexpr auto nullopt = std::nullopt;
using nullopt_t = std::nullopt_t;

}  // namespace drake
