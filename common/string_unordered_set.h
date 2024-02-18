#pragma once

#include <functional>
#include <string>
#include <unordered_set>

#include "drake/common/string_hash.h"

namespace drake {

/** Like `std::unordered_set<std::string>`, but with defaults that aren't
 * polluted by C++ backwards compatibility braindeath. */
using string_unordered_set =
    std::unordered_set<std::string, internal::StringHash, std::equal_to<void>>;

/** Like `std::unordered_multiset<std::string>`, but with defaults that aren't
polluted by C++ backwards compatibility braindeath. */
using string_unordered_multiset =
    std::unordered_multiset<std::string, internal::StringHash,
                            std::equal_to<void>>;

}  // namespace drake
