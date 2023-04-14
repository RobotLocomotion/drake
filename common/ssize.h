#pragma once

// Many C++20 std headers define ssize; pick one. This is needed even to
// get the __cpp_lib_ssize define set.
#include <vector>

#ifdef __cpp_lib_ssize

using std::ssize;

#else  // __cpp_lib_ssize

#include <cstddef>      // For std::ptrdiff_t
#include <type_traits>  // For std::common_type_t, std::make_signed_t

namespace drake {

/** Implements C++20 %std::ssize() for earlier compilers. See
https://en.cppreference.com/w/cpp/iterator/size for documentation. Will be
removed once all Drake-supported platforms offer %std::ssize(). */

// The implementations below are taken directly from the cppreference
// documentation. BTW std::size() is already supported on all Drake platforms.

template <class C>
constexpr auto ssize(const C& c)
    -> std::common_type_t<std::ptrdiff_t,
                          std::make_signed_t<decltype(c.size())> > {
  using R = std::common_type_t<std::ptrdiff_t,
                               std::make_signed_t<decltype(c.size())> >;
  return static_cast<R>(c.size());
}

/** This signature returns the size of built-in (C style) arrays. */
template <class T, std::ptrdiff_t N>
constexpr std::ptrdiff_t ssize(const T (&array)[N]) noexcept {
  return N;
}

}  // namespace drake

#endif  // __cpp_lib_ssize
