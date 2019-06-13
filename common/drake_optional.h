#pragma once

/// @file
/// Provides drake::optional as an alias for the appropriate implementation of
/// std::optional or stx::optional for the C++ toolchain being used.

// If the current toolchain offers a working std::optional, then use it.  If
// std::optional does not exist, do not use std::experimental::optional (even
// though STX would otherwise do so by default) because its API often differs
// from std::optional.
#define STX_NO_STD_OPTIONAL
#if __cplusplus >= 201703L
#if defined(__has_include)
#if __has_include(<optional>)
#include <optional>
#undef STX_NO_STD_OPTIONAL
#endif
#endif
#endif

// If the current toolchain offers a working std::variant, then use it.
#define STX_NO_STD_VARIANT
#if __cplusplus >= 201703L
#if defined(__has_include)
#if __has_include(<variant>)
#include <variant>
#undef STX_NO_STD_VARIANT
#endif
#endif
#endif

// Due to https://github.com/tcbrindle/cpp17_headers/issues/4, we must always
// include variant before optional.  Rather than leaving that up to our users,
// we force the issue here.  Note that this does NOT place stx::variant into
// the drake namespace, so users still need to include drake_variant.h in order
// to use variants.
//
// Furthermore, the stx header is broken in various ways, so we have to do
// work-arounds after including it.
#ifdef STX_NO_STD_VARIANT
#include <stx/variant.hpp>
#define STX_HAVE_IN_PLACE_T 1
#endif
#ifdef STX_NO_STD_OPTIONAL
#include <stx/optional.hpp>
#endif

namespace drake {

#ifdef STX_NO_STD_OPTIONAL
template <typename T>
using optional = stx::optional<T>;
constexpr auto nullopt = stx::nullopt;
using nullopt_t = stx::nullopt_t;
#else
template <typename T>
using optional = std::optional<T>;
constexpr auto nullopt = std::nullopt;
using nullopt_t = std::nullopt_t;
#endif

}  // namespace drake
