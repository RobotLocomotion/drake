#pragma once

#include <filesystem>
#include <string>
#include <tuple>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace internal {

// Resolves the full path of a URI. If @p uri starts with "package:" or
// "model:", the ROS packages specified in @p package_map are searched.
// Otherwise, iff a root_dir was provided then @p uri is appended to the end
// of @p root_dir (if it's not already an absolute path) and checked for
// existence.  If the file does not exist or is not found, an error is posted
// to @p diagnostic. The returned path will be lexically normalized. In other
// words, a path like `/some//path/to/ignored/../file.txt` (with duplicate
// slashes, directory changes, etc.) would be boiled down to
// `/some/path/to/file.txt`.
//
// @param diagnostic The error-reporting channel.
//
// @param[in] uri The name of the resource to find.
//
// @param[in] package_map A map where the keys are ROS package names and the
// values are the paths to the packages. This is only used if @p filename
// starts with "package:"or "model:".
//
// @param[in] root_dir The root directory to look in. This is only used when
// @p filename does not start with "package:".  Can be empty when only URIs
// (not relative paths) should be allowed for @p uri.
//
// @return The file's full path, lexically normalized, and weather the file
// exists.
std::tuple<std::string, bool> ResolveUri(
                       const drake::internal::DiagnosticPolicy& diagnostic,
                       const std::string& uri,
                       const PackageMap& package_map,
                       const std::string& root_dir);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
