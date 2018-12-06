#pragma once

// Due to https://github.com/tcbrindle/cpp17_headers/issues/4, we must always
// include variant before optional.  Rather than leaving that up to our users,
// we force the issue here.  Note that this does NOT place stx::variant into
// the drake namespace, so users still need to include drake_variant.h in order
// to use variants.
//
// Furthermore, the stx header is broken in various ways, so we have to do some
// work-arounds before and after including it.
#if defined(__has_include)
#if __has_include(<variant>)
#include <variant>
#endif
#endif
#include <stx/variant.hpp>
#define STX_HAVE_IN_PLACE_T 1

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
