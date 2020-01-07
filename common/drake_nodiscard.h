#pragma once

// NOLINTNEXTLINE
#warning DRAKE DEPRECATED: The drake/common/drake_nodiscard.h header is being removed from Drake on or after 2020-05-01.  Instead, developers should use [[nodiscard]] directly.

#if defined(DRAKE_DOXYGEN_CXX) || __has_cpp_attribute(nodiscard)
/** Synonym for [[nodiscard]], iff the current compiler supports it;
see https://en.cppreference.com/w/cpp/language/attributes/nodiscard. */
// NOLINTNEXTLINE(whitespace/braces)
#define DRAKE_NODISCARD [[nodiscard]]
#else
#define DRAKE_NODISCARD
#endif
