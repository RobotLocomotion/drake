#pragma once

// As of our currently supported platforms (Ubuntu 16.04 Xenial and macOS 10.13
// High Sierra), the std::experimental::optional implementations for libstdc++
// and libc++ lack some C++17 features, such as std::optional::has_value() or
// std::bad_optional_acces.  Thus, we'll force-enable the STX implementation
// for now.  In the future if some new platform has C++17 support, we'll want
// to allow this to fallback to use it.
#define STX_NO_STD_OPTIONAL

#include <stx/optional.hpp>

/// @file
/// Provides drake::optional as an alias for the appropriate implementation of
/// std::optional or std::experimental::optional or stx::optional for the C++
/// toolchain being used.  (The alias is selected preferentially in that order,
/// so the most widely-compatible implementation will always be used.)

namespace drake {

template <typename T>
using optional = stx::optional<T>;

constexpr auto nullopt = stx::nullopt;

}  // namespace drake
