#pragma once

#include <string>
#include <string_view>

namespace drake {
namespace internal {

/* StringHash allows us unordered maps and sets with `std::string` keys to
accept either `std::string` or `std::string_view` or `const char*` key during
lookup operations (find, count, etc.) via "transparent" lookups. */
struct StringHash final : public std::hash<std::string>,
                          public std::hash<std::string_view> {
  using is_transparent = void;

  using std::hash<std::string>::operator();
  using std::hash<std::string_view>::operator();
  auto operator()(const char* key) const {
    return this->operator()(std::string_view{key});
  }
};

}  // namespace internal
}  // namespace drake
