#include "drake/multibody/multibody_tree/parsing/parser_path_utils.h"

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

// The unit test that most directly covers this method is:
// drake/attic/multibody/parsers/test/urdf_parser_test/urdf_parser_test.cc.
// TODO(jwnimmer-tri) Port a more direct unit test to this package, so we
// are not relying on attic code to test this non-attic function.
string ResolveFilename(const string& filename, const PackageMap& package_map,
                       const string& root_dir) {
  spruce::path full_filename_spruce;
  spruce::path raw_filename_spruce(filename);
  const std::vector<string> split_filename = raw_filename_spruce.split();

  if (IsAbsolutePath(filename)) {
    if (!raw_filename_spruce.exists()) {
      drake::log()->warn("File {} could not be found.", filename);
      return string();
    }
    return filename;
  } else if (split_filename.front() == "package:") {
    // A correctly formatted filename is:
    //
    //   package://package_name/bar/baz/model.xyz
    //
    // Thus, index 0 contains "package", index 1 contains "", and index 2
    // contains the package name. Furthermore, since the model file must follow
    // the package name, there must be at least 4 tokens.
    const int kMinNumTokens = 4;
    const int kPackageNameIndex = 2;
    DRAKE_DEMAND(split_filename.size() >= kMinNumTokens);

    string package_path_string;
    if (GetPackagePath(split_filename.at(kPackageNameIndex), package_map,
        &package_path_string)) {
      full_filename_spruce = spruce::path(package_path_string);

      // The following loop starts at index 3 to skip the "package", "", and
      // [package name] tokens as described above.
      for (int i = kPackageNameIndex + 1;
          i < static_cast<int>(split_filename.size()); ++i) {
        full_filename_spruce.append(split_filename.at(i));
      }
    } else {
      return string();
    }
  } else {
    const string normalized_root_dir = spruce::path(root_dir).getStr();

    // If root_dir is a relative path, convert it to an absolute path.
    if (!IsAbsolutePath(normalized_root_dir)) {
      full_filename_spruce = spruce::dir::getcwd();
      full_filename_spruce.append(normalized_root_dir);
    } else {
      full_filename_spruce = spruce::path(normalized_root_dir);
    }

    full_filename_spruce.append(filename);
  }

  if (!full_filename_spruce.exists()) {
    drake::log()->warn("File {} resolved to {} and could not be found.",
                       filename, full_filename_spruce.getStr());
    return string();
  }
  return full_filename_spruce.getStr();
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
