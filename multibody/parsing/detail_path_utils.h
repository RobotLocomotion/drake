#pragma once

#include <filesystem>
#include <string>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace internal {

// The return type of ResolveUri().
struct ResolveUriResult {
  // When `exists` is true, returns `full_path` as a string.
  // When `exists` is false, returns an empty string.
  std::string GetStringPathIfExists() const;

  std::filesystem::path full_path;
  bool exists{};
};

// Resolves the full path of a URI. If `uri` starts with "package:" or "model:",
// the ROS packages specified in `package_map` are searched. Otherwise, iff a
// root_dir was provided then `uri` is appended to the end of `root_dir` (if
// it's not already an absolute path).
//
// The `result.full_path` will be lexically normalized. In other words, a path
// like `/some//path/to/ignored/../file.txt` (with duplicate slashes, directory
// changes, etc.) would be boiled down to `/some/path/to/file.txt`. The path
// will be returned even when the file does not exist.
//
// If the file does not exist, an error will be posted to `diagnostic` and
// `result.exists` will be `false`.
//
// @param diagnostic The error-reporting channel.
//
// @param[in] uri The name of the resource to find.
//
// @param[in] package_map A map where the keys are ROS package names and the
// values are the paths to the packages. This is only used if `filename`,
// starts with "package:"or "model:".
//
// @param[in] root_dir The root directory to look in. This is only used when
// `filename` does not start with "package:".  Can be empty when only URIs
// (not relative paths) should be allowed for `uri`.
//
// @return The file's full path, lexically normalized, and whether it exists.
ResolveUriResult ResolveUri(const drake::internal::DiagnosticPolicy& diagnostic,
                            const std::string& uri,
                            const PackageMap& package_map,
                            const std::string& root_dir);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
