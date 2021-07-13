#pragma once

#include <cstring>
#include <type_traits>

namespace drake {
namespace internal {

/** Re-interprets the bits of type `From` as type `To`.
This is similar to https://en.cppreference.com/w/cpp/numeric/bit_cast, but we
need to roll our own until we have C++20 available. */
template <class To, class From>
To bit_cast(const From& from) noexcept {
  static_assert(std::is_trivially_constructible_v<To>);
  To result;
  static_assert(sizeof(To) == sizeof(From));
  static_assert(std::is_trivially_copyable_v<To>);
  static_assert(std::is_trivially_copyable_v<From>);
  std::memcpy(&result, &from, sizeof(result));
  return result;
}

}  // namespace internal
}  // namespace drake
