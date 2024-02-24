#pragma once

#include <functional>
#include <map>
#include <string>

namespace drake {

/** Like `std::map<std::string, T>`, but with better defaults than the plain
`std::map<std::string, T>` spelling. We need `std::less<void>` as the comparison
function so that `std::string_view` and `const char*` can be used as lookup keys
without copying them to a `std::string`. */
template <typename T>
using string_map = std::map<std::string, T, std::less<void>>;

/** Like `std::multimap<std::string, T>`, but with better defaults than the
plain `std::multimap<std::string, T>` spelling. We need `std::less<void>` as the
comparison function so that `std::string_view` and `const char*` can be used as
lookup keys without copying them to a `std::string`. */
template <typename T>
using string_multimap = std::multimap<std::string, T, std::less<void>>;

}  // namespace drake
