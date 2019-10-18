#pragma once

// TODO(jwnimmer-tri) Once we are in --std=c++17 mode as our minimum version,
// we can remove this file and just say [[nodiscard]] directly everywhere.

#if defined(DRAKE_DOXYGEN_CXX) || __has_cpp_attribute(nodiscard)
/** Synonym for [[nodiscard]], iff the current compiler supports it;
see https://en.cppreference.com/w/cpp/language/attributes/nodiscard. */
// NOLINTNEXTLINE(whitespace/braces)
#define DRAKE_NODISCARD [[nodiscard]]
#else
#define DRAKE_NODISCARD
#endif
