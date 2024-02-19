#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "drake/common/string_hash.h"

namespace drake {

/** Like `std::unordered_map<std::string, T>`, but with better defaults than the
plain `std::unordered_map<std::string, T>` spelling. We need the custom hash and
comparison functions so that `std::string_view` and `const char*` can be used as
lookup keys without copying them to a `std::string`. */
template <typename T>
using string_unordered_map =
    std::unordered_map<std::string, T, internal::StringHash,
                       std::equal_to<void>>;

/** Like `std::unordered_multimap<std::string, T>`, but with better defaults
than the plain `std::unordered_multimap<std::string, T>` spelling. We need the
custom hash and comparison functions so that `std::string_view` and `const
char*` can be used as lookup keys without copying them to a `std::string`. */
template <typename T>
using string_unordered_multimap =
    std::unordered_multimap<std::string, T, internal::StringHash,
                            std::equal_to<void>>;

}  // namespace drake
