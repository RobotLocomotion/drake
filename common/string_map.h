#pragma once

#include <functional>
#include <map>
#include <string>

namespace drake {

/** Like `std::map<std::string, T>`, but with defaults that aren't polluted by
C++ backwards compatibility braindeath. */
template <typename T>
using string_map = std::map<std::string, T, std::less<void>>;

/** Like `std::multimap<std::string, T>`, but with defaults that aren't polluted
by C++ backwards compatibility braindeath. */
template <typename T>
using string_multimap = std::multimap<std::string, T, std::less<void>>;

}  // namespace drake
