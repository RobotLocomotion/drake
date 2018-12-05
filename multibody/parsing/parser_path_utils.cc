#include "drake/multibody/parsing/parser_path_utils.h"

#include <string>
#include <vector>

#include <spruce.hh>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

using std::string;

namespace drake {
namespace multibody {
namespace parsing {
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
// Searches for key package in package_map. If the key exists, this saves the
// associated value in the string pointed to by package_path and then returns
// true. It returns false otherwise.
bool GetPackagePath(const string& package, const PackageMap& package_map,
                    string* package_path) {
  if (package_map.Contains(package)) {
    *package_path = package_map.GetPath(package);
    return true;
  } else {
    drake::log()->warn("Couldn't find package '{}' in the supplied package"
                       "path.", package);
    return false;
  }
}
}  // namespace

string ResolveUri(const string& uri, const PackageMap& package_map,
                       const string& root_dir) {
  spruce::path full_filename_spruce;
  spruce::path raw_filename_spruce(uri);
  const std::vector<string> split_filename = raw_filename_spruce.split();

  // Look for a URI with an explicit scheme.
  if (uri.find("://") != std::string::npos) {
    if (split_filename.front() == "file:") {
      // Correctly formatted URI in this format is:
      //
      //   file://bar/baz/model.xyz
      //
      // Thus, index 0 contains "file", index 1 contains "", and
      // indices 3 onward contain the path.
      const int kMinNumTokens = 3;
      const int kFileNameIndex = 2;
      DRAKE_DEMAND(split_filename.size() >= kMinNumTokens);
      for (int i = kFileNameIndex;
           i < static_cast<int>(split_filename.size()); ++i) {
        full_filename_spruce.append(split_filename.at(i));
      }
    } else if (split_filename.front() == "package:" ||
        split_filename.front() == "model:") {
      // A correctly formatted URI in this format is:
      //
      //   package://package_name/bar/baz/model.xyz or
      //   model://package_name/bar/baz/model.xyz
      //
      // Thus, index 0 contains "package:" or "model:", index 1 contains "", and
      // index 2 contains the package name. Furthermore, since the model file
      // must follow the package name, there must be at least 4 tokens.
      const int kMinNumTokens = 4;
      const int kPackageNameIndex = 2;
      DRAKE_DEMAND(split_filename.size() >= kMinNumTokens);

      string package_path_string;
      if (GetPackagePath(split_filename.at(kPackageNameIndex), package_map,
                         &package_path_string)) {
        full_filename_spruce = spruce::path(package_path_string);

        // The following loop starts at index 3 to skip the "package:"
        // (or "model:"), "", and [package name] tokens as described above.
        for (int i = kPackageNameIndex + 1;
             i < static_cast<int>(split_filename.size()); ++i) {
          full_filename_spruce.append(split_filename.at(i));
        }
      } else {
        // The specified package was not found in the PackageMap.
        return string();
      }

    } else {
      const auto error_message = "URI specifies an unsupported scheme. "
          "Supported schemes are 'file://', 'model://', and 'package://'. "
          "Provided URI: " + uri;
      throw std::runtime_error(error_message);
    }
  }

  // Strictly speaking, a URI should not just be a filename (i.e., it should
  // be preceded by "file://"). But we allow this for backward compatibility
  // and user convenience.

  // Try treating the URI as an absolute path first.
  if (IsAbsolutePath(uri)) {
    if (!raw_filename_spruce.exists()) {
      drake::log()->warn("File {} could not be found.", uri);
      return string();
    }
    return uri;
  } else {
    // Try it as a normalized root directory.
    const string normalized_root_dir = spruce::path(root_dir).getStr();

    // If root_dir is a relative path, convert it to an absolute path.
    if (!IsAbsolutePath(normalized_root_dir)) {
      full_filename_spruce = spruce::dir::getcwd();
      full_filename_spruce.append(normalized_root_dir);
    } else {
      full_filename_spruce = spruce::path(normalized_root_dir);
    }

    full_filename_spruce.append(uri);
  }

  if (!full_filename_spruce.exists()) {
    drake::log()->warn("File {} resolved to {} and could not be found.",
                       uri, full_filename_spruce.getStr());
    return string();
  }
  return full_filename_spruce.getStr();
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
