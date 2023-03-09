#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace drake {
namespace internal {

/* The return type of FindOrCreateDrakeCache(). Exactly one of the two strings
is non-empty. */
struct PathOrError {
  /** The absolute path to a writeable, Drake-specific cache directory. */
  std::filesystem::path abspath;
  /** The error message. */
  std::string error;
};

/* Returns the platform-appropriate location for ~/.cache/drake/{subdir},
creating it if necessary. */
PathOrError FindOrCreateCache(std::string_view subdir);

}  // namespace internal
}  // namespace drake
