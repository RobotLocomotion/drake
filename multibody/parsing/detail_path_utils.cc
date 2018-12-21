#include "drake/multibody/parsing/detail_path_utils.h"

#include <regex>
#include <string>
#include <vector>

#include <spruce.hh>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace detail {

using std::string;

namespace {
bool IsAbsolutePath(const string& filename) {
  const string prefix = "/";
  return filename.substr(0, prefix.size()) == prefix;
}
}  // namespace

string GetFullPath(const string& file_name) {
  string result = file_name;
  if (result.empty()) {
    throw std::runtime_error("drake::parsers::GetFullPath: ERROR: file_name is "
                             "empty.");
  }

  if (IsAbsolutePath(result)) {
    // The specified file is already an absolute path. The following code
    // verifies that the file exists.
    spruce::path path(file_name);
    if (!path.isFile()) {
      throw std::runtime_error("drake::parsers::GetFullPath: ERROR: "
          "file_name \"" + file_name + "\" is not a file.");
    }
  } else {
    // The specified file is a relative path. The following code obtains the
    // full path and verifies that the file exists.
    spruce::path path = spruce::dir::getcwd();
    path.append(file_name);
    if (path.isFile()) {
      result = path.getStr();
    } else {
      throw std::runtime_error("drake::parsers::GetFullPath: ERROR: "
          "file_name \"" + file_name + "\" is not a file or does not exist.");
    }
  }
  return result;
}

namespace {
// Searches for key package in package_map. If the key exists, returns the
// associated value; else prints a warning and returns nullopt.
optional<string> GetPackagePath(
    const string& package, const PackageMap& package_map) {
  if (package_map.Contains(package)) {
    return package_map.GetPath(package);
  } else {
    drake::log()->warn("Couldn't find package '{}' in the supplied package"
                       "path.", package);
    return nullopt;
  }
}
}  // namespace

string ResolveUri(const string& uri, const PackageMap& package_map,
                  const string& root_dir) {
  spruce::path result;

  // Parse the given URI into pieces.
  static const never_destroyed<std::regex> uri_matcher{
    "^([a-z0-9+.-]+)://([^/]*)(/+.*)"
  };
  std::smatch match;
  if (std::regex_match(uri, match, uri_matcher.access())) {
    // The `uri` was actually a URI (not a bare filename).
    DRAKE_DEMAND(match.size() == 4);
    const auto& uri_scheme = match[1];
    const auto& uri_package = match[2];
    const auto& uri_path = match[3];  // This includes the leading '/' (if any).
    if (uri_scheme == "file") {
      result = uri_path.str();
    } else if ((uri_scheme == "model") || (uri_scheme == "package")) {
      optional<string> package_path =
          GetPackagePath(uri_package, package_map);
      if (!package_path) { return {}; }
      result = spruce::path(*package_path);
      result.append(uri_path.str());
    } else {
      drake::log()->warn(
          "URI '{}' specifies an unsupported scheme; supported schemes are "
          "'file://', 'model://', and 'package://'.", uri);
      return {};
    }
  } else {
    // Strictly speaking a URI should not just be a bare filename, but we allow
    // this for backward compatibility and user convenience.
    const string& filename = uri;
    if (IsAbsolutePath(filename)) {
      result = filename;
    } else if (IsAbsolutePath(root_dir)) {
      result = root_dir;
      result.append(filename);
    } else {
      result = spruce::dir::getcwd();
      result.append(root_dir);
      result.append(filename);
    }
  }

  if (!result.exists()) {
    drake::log()->warn("URI '{}' resolved to '{}' which could not be found.",
                       uri, result.getStr());
    return {};
  }
  return result.getStr();
}

}  // namespace detail
}  // namespace multibody
}  // namespace drake
