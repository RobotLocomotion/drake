#pragma once

#include <string>

#include <fmt/format.h>

namespace drake {

/** Utility function for dereferencing a pointer with a runtime check against
 being null.

 Note: if a pointer to a const type is passed in, a const reference comes out,
 otherwise a non-const reference comes out. */
template <typename T>
T& SafeDereference(std::string_view variable_name, T* ptr) {
  if (ptr == nullptr) {
    throw std::runtime_error(
        fmt::format("Condition '{} != nullptr' failed.", variable_name));
  }
  return *ptr;
}

}  // namespace drake
