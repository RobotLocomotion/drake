#pragma once

#include <ranges>

namespace drake {

/** Concept that constrains the range R to be a std::ranges::view where the
range's value type is exactly T. This enables documenting the return type of
range-returning functions, without promising a concrete type of range.

This concept should not be used by code outside of Drake (it is not Stable API);
it is intended only for use by Drake's documentation. */
template <typename R, typename T>
concept range_view_of =
    std::ranges::view<R> && std::same_as<std::ranges::range_value_t<R>, T>;

}  // namespace drake
