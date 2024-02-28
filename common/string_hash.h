#pragma once

#include <string>
#include <string_view>

namespace drake {
namespace internal {

/* StringHash allows us unordered maps and sets with `std::string` keys to
accept either `std::string` or `std::string_view` or `const char*` key during
lookup operations (find, count, etc.) via "transparent" lookups. */
struct StringHash final : public std::hash<std::string_view> {
  using is_transparent = void;

  // Note that the call operator `operator()(std::string_view)` is inherited
  // from our base class. We don't need to add overloads for `std::string` or
  // `const char*` because those implicitly convert to `string_view`.
};

}  // namespace internal
}  // namespace drake
