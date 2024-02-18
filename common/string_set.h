#pragma once

#include <functional>
#include <set>
#include <string>

namespace drake {

/** Like `std::set<std::string>`, but with defaults that aren't polluted by C++
backwards compatibility braindeath. */
using string_set = std::set<std::string, std::less<void>>;

/** Like `std::multiset<std::string>`, but with defaults that aren't polluted by
C++ backwards compatibility braindeath. */
using string_multiset = std::multiset<std::string, std::less<void>>;

}  // namespace drake
