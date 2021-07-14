#pragma once

#include <cstring>
#include <type_traits>

namespace drake {
namespace internal {

/** Implements C++20 https://en.cppreference.com/w/cpp/numeric/bit_cast (but
without the overload resolution guards, which are not necessary in our case.)
(Once all of Drake's supported platforms offer std::bit_cast, we can remove
this function in lieu of the std one.) */
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
