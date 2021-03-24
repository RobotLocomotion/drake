#pragma once

#include <string>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace internal {

// Obtains the full path of @file_name. If @p file_name is already a
// full path (i.e., it starts with a "/"), the path is not modified.
// If @p file_name is a relative path, this method converts it into
// an absolute path based on the current working directory.
//
// @throws std::runtime_error if the file does not exist or if @p
// file_name is empty.
std::string GetFullPath(const std::string& file_name);

// Implementations for PackageMap::ResolveUri.
FindResourceResult ResolveUriToResult(
    const std::string& uri, const PackageMap& package_map,
    const std::string& root_dir);
std::string ResolveUri(const std::string& uri,
                       const PackageMap& package_map,
                       const std::string& root_dir);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
