#pragma once

#include <string>

#include "drake/multibody/multibody_tree/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Obtains the full path of @file_name. If @p file_name is already a
/// full path (i.e., it starts with a "/"), the path is not modified.
/// If @p file_name is a relative path, this method converts it into
/// an absolute path based on the current working directory.
///
/// @throws std::runtime_error if the file does not exist or if @p
/// file_name is empty.
std::string GetFullPath(const std::string& file_name);

/// Resolves the full path of a file. If @p filename starts with
/// "package:", the ROS packages specified in @p package_map are
/// searched.  Otherwise, @p filename is appended to the end of @p
/// root_dir (if it's not already an absolute path) and checked for
/// existence. If the file does not exist or is not found, a warning
/// is printed to `std::cerr` and an empty string is returned.
///
/// @param[in] filename The name of the file to find.
///
/// @param[in] package_map A map where the keys are ROS package names and the
/// values are the paths to the packages. This is only used if @p filename
/// starts with "package:".
///
/// @param[in] root_dir The root directory to look in. This is only used when
/// @p filename does not start with "package:".
///
/// @return The file's full path or an empty string if the file is not
/// found or does not exist.
std::string ResolveFilename(const std::string& filename,
                            const PackageMap& package_map,
                            const std::string& root_dir);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
