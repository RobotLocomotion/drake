#pragma once

#include <functional>
#include <string>
#include <unordered_set>

#include "drake/common/string_hash.h"

namespace drake {

/** Like `std::unordered_set<std::string>`, but with better defaults than the
plain `std::unordered_set<std::string>` spelling. We need the custom hash and
comparison functions so that `std::string_view` and `const char*` can be used
as lookup keys without copying them to a `std::string`. */
using string_unordered_set =
    std::unordered_set<std::string, internal::StringHash, std::equal_to<void>>;

/** Like `std::unordered_multiset<std::string>`, but with better defaults than
the plain `std::unordered_multiset<std::string>` spelling. We need the custom
hash and comparison functions so that `std::string_view` and `const char*` can
be used as lookup keys without copying them to a `std::string`. */
using string_unordered_multiset =
    std::unordered_multiset<std::string, internal::StringHash,
                            std::equal_to<void>>;

}  // namespace drake
