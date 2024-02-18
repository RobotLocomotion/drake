#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "drake/common/string_hash.h"

namespace drake {

/** Like `std::unordered_map<std::string, T>`, but with defaults that aren't
polluted by C++ backwards compatibility braindeath. */
template <typename T>
using string_unordered_map =
    std::unordered_map<std::string, T, internal::StringHash,
                       std::equal_to<void>>;

/** Like `std::unordered_multimap<std::string, T>`, but with defaults that
aren't polluted by C++ backwards compatibility braindeath. */
template <typename T>
using string_unordered_multimap =
    std::unordered_multimap<std::string, T, internal::StringHash,
                            std::equal_to<void>>;

}  // namespace drake
