#pragma once

#include <functional>
#include <set>
#include <string>

namespace drake {

/** Like `std::set<std::string>`, but with better defaults than the plain
`std::set<std::string>` spelling. We need `std::less<void>` as the comparison
function so that `std::string_view` and `const char*` can be used as lookup keys
without copying them to a `std::string`. */
using string_set = std::set<std::string, std::less<void>>;

/** Like `std::multiset<std::string>`, but with better defaults than the plain
`std::multiset<std::string>` spelling. We need `std::less<void>` as the
comparison function so that `std::string_view` and `const char*` can be used as
lookup keys without copying them to a `std::string`. */
using string_multiset = std::multiset<std::string, std::less<void>>;

}  // namespace drake
