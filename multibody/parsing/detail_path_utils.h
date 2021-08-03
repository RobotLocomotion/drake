#pragma once

#include <string>

#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace internal {

// Obtains the full path of @file_name. If @p file_name is already a
// full path (i.e., it starts with a "/"), the path is not modified.
// If @p file_name is a relative path, this method converts it into
// an absolute path based on the current working directory.
//
// @throws std::exception if the file does not exist or if @p
// file_name is empty.
std::string GetFullPath(const std::string& file_name);

// Resolves the full path of a URI. If @p uri starts with "package:" or
// "model:", the ROS packages specified in @p package_map are searched.
// Otherwise, iff a root_dir was provided then @p uri is appended to the end
// of @p root_dir (if it's not already an absolute path) and checked for
// existence.  If the file does not exist or is not found, a warning is
// printed to `std::cerr` and an empty string is returned. The returned path
// will be lexically normalized. In other words, a path like
// `/some//path/to/ignored/../file.txt` (with duplicate slashes, directory
// changes, etc.) would be boiled down to `/some/path/to/file.txt`.
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
// @return The file's full path, lexically normalized, or an empty string if
// the file is not found or does not exist.
std::string ResolveUri(const std::string& uri,
                       const PackageMap& package_map,
                       const std::string& root_dir);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
