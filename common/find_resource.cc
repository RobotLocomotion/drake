#include "drake/common/find_resource.h"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_marker.h"
#include "drake/common/find_loaded_library.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

using std::getenv;
using std::string;

namespace drake {

namespace fs = std::filesystem;

using Result = FindResourceResult;

std::optional<string> Result::get_absolute_path() const {
  return absolute_path_;
}

string Result::get_absolute_path_or_throw() const {
  // If we have a path, return it.
  const auto& optional_path = get_absolute_path();
  if (optional_path) {
    return *optional_path;
  }

  // Otherwise, throw the error message.
  const auto& optional_error = get_error_message();
  DRAKE_ASSERT(optional_error != std::nullopt);
  throw std::runtime_error(*optional_error);
}

std::optional<string> Result::get_error_message() const {
  // If an error has been set, return it.
  if (error_message_ != std::nullopt) {
    DRAKE_ASSERT(absolute_path_ == std::nullopt);
    return error_message_;
  }

  // If successful, return no-error.
  if (absolute_path_ != std::nullopt) {
    return std::nullopt;
  }

  // Both optionals are null; we are empty; return a default error message.
  DRAKE_ASSERT(resource_path_.empty());
  return string("No resource was requested (empty result)");
}

string Result::get_resource_path() const {
  return resource_path_;
}

Result Result::make_success(string resource_path, string absolute_path) {
  DRAKE_THROW_UNLESS(!resource_path.empty());
  DRAKE_THROW_UNLESS(!absolute_path.empty());

  Result result;
  result.resource_path_ = std::move(resource_path);
  result.absolute_path_ = std::move(absolute_path);
  result.CheckInvariants();
  return result;
}

Result Result::make_error(string resource_path, string error_message) {
  DRAKE_THROW_UNLESS(!resource_path.empty());
  DRAKE_THROW_UNLESS(!error_message.empty());

  Result result;
  result.resource_path_ = std::move(resource_path);
  result.error_message_ = std::move(error_message);
  result.CheckInvariants();
  return result;
}

Result Result::make_empty() {
  Result result;
  result.CheckInvariants();
  return result;
}

void Result::CheckInvariants() {
  if (resource_path_.empty()) {
    // For our "empty" state, both success and error must be empty.
    DRAKE_DEMAND(absolute_path_ == std::nullopt);
    DRAKE_DEMAND(error_message_ == std::nullopt);
  } else {
    // For our "non-empty" state, we must have exactly one of success or error.
    DRAKE_DEMAND((absolute_path_ == std::nullopt) !=
                 (error_message_ == std::nullopt));
  }
  // When non-nullopt, the path and error cannot be the empty string.
  DRAKE_DEMAND((absolute_path_ == std::nullopt) || !absolute_path_->empty());
  DRAKE_DEMAND((error_message_ == std::nullopt) || !error_message_->empty());
}

namespace {

// A valid resource root will always contain this file.
const char* const kSentinelRelpath = "drake/.drake-find_resource-sentinel";

// Returns true iff the path is relative (not absolute nor empty).
bool IsRelativePath(const string& path) {
  return !path.empty() && (path[0] != '/');
}

// Taking `root` to be Drake's resource root, confirm that the sentinel file
// exists and return the found resource_path (or an error if either the
// sentinel or resource_path was missing).
Result CheckAndMakeResult(const string& root_description, const string& root,
                          const string& resource_path) {
  DRAKE_DEMAND(!root_description.empty());
  DRAKE_DEMAND(!root.empty());
  DRAKE_DEMAND(!resource_path.empty());
  DRAKE_DEMAND(fs::is_directory({root}));
  DRAKE_DEMAND(IsRelativePath(resource_path));

  // Check for the sentinel.
  if (!fs::is_regular_file({root + "/" + kSentinelRelpath})) {
    return Result::make_error(
        resource_path,
        fmt::format(
            "Could not find Drake resource_path '{}' because {} specified a "
            "resource root of '{}' but that root did not contain the expected "
            "sentinel file '{}'.",
            resource_path, root_description, root, kSentinelRelpath));
  }

  // Check for the resource_path.
  const string abspath = root + '/' + resource_path;
  if (!fs::is_regular_file({abspath})) {
    return Result::make_error(
        resource_path,
        fmt::format(
            "Could not find Drake resource_path '{}' because {} specified a "
            "resource root of '{}' but that root did not contain the expected "
            "file '{}'.",
            resource_path, root_description, root, abspath));
  }

  return Result::make_success(resource_path, abspath);
}

// If the DRAKE_RESOURCE_ROOT environment variable is usable as a resource
// root, returns its value, else returns nullopt.
std::optional<string> MaybeGetEnvironmentResourceRoot() {
  const char* const env_name = kDrakeResourceRootEnvironmentVariableName;
  const char* const env_value = getenv(env_name);
  if (!env_value) {
    log()->debug("FindResource ignoring {} because it is not set.", env_name);
    return std::nullopt;
  }
  const std::string root{env_value};
  if (!fs::is_directory({root})) {
    static const logging::Warn log_once(
        "FindResource ignoring {}='{}' because it does not exist.", env_name,
        env_value);
    return std::nullopt;
  }
  if (!fs::is_directory({root + "/drake"})) {
    static const logging::Warn log_once(
        "FindResource ignoring {}='{}' because it does not contain a 'drake' "
        "subdirectory.",
        env_name, env_value);
    return std::nullopt;
  }
  if (!fs::is_regular_file({root + "/" + kSentinelRelpath})) {
    static const logging::Warn log_once(
        "FindResource ignoring {}='{}' because it does not contain the "
        "expected sentinel file '{}'.",
        env_name, env_value, kSentinelRelpath);
    return std::nullopt;
  }
  return root;
}

// If we are linked against libdrake_marker.so, and the install-tree-relative
// path resolves correctly, returns the install tree resource root, else
// returns nullopt.
std::optional<string> MaybeGetInstallResourceRoot() {
  // Ensure that we have the library loaded.
  DRAKE_DEMAND(drake::internal::drake_marker_lib_check() == 1234);
  std::optional<string> maybe_libdrake_dir =
      LoadedLibraryPath("libdrake_marker.so");
  if (maybe_libdrake_dir) {
    log()->debug("FindResource libdrake_dir='{}'", *maybe_libdrake_dir);
    const fs::path libdrake_dir{*maybe_libdrake_dir};
    const fs::path root = libdrake_dir / "../share";
    if (fs::is_directory(root)) {
      return root.string();
    }
    const fs::path canonical_root =
        fs::canonical(libdrake_dir / "libdrake_marker.so")
            .parent_path()
            .parent_path() /
        "share";
    if (fs::is_directory(canonical_root)) {
      return canonical_root.string();
    }
    log()->debug(
        "FindResource ignoring CMake install candidate '{}' ('{}') because it "
        "does not exist",
        root.string(), canonical_root.string());
  } else {
    log()->debug("FindResource has no CMake install candidate");
  }
  return std::nullopt;
}

Result MakeResultFrom(const string& resource_path, RlocationOrError other) {
  return other.error.empty()
             ? Result::make_success(resource_path, std::move(other.abspath))
             : Result::make_error(resource_path, std::move(other.error));
}

}  // namespace

const char* const kDrakeResourceRootEnvironmentVariableName =
    "DRAKE_RESOURCE_ROOT";

Result FindResource(const string& resource_path) {
  // Check if resource_path is well-formed: a relative path that starts with
  // "drake" as its first directory name.  A valid example would look like:
  // "drake/common/test/find_resource_test_data.txt".  Requiring strings passed
  // to drake::FindResource to start with "drake" is redundant, but preserves
  // compatibility with the original semantics of this function; if we want to
  // offer a function that takes paths without "drake", we can use a new name.
  if (!IsRelativePath(resource_path)) {
    return Result::make_error(
        resource_path,
        fmt::format("Drake resource_path '{}' is not a relative path.",
                    resource_path));
  }
  const string prefix("drake/");
  if (!resource_path.starts_with(prefix)) {
    return Result::make_error(
        resource_path,
        fmt::format("Drake resource_path '{}' does not start with {}.",
                    resource_path, prefix));
  }

  // We will check each potential resource root one by one.  The first root
  // that is present will be chosen, even if does not contain the particular
  // resource_path.  We expect that all sources offer all files.

  // (1) Check the environment variable.
  if (auto guess = MaybeGetEnvironmentResourceRoot()) {
    return CheckAndMakeResult(
        fmt::format("{} environment variable",
                    kDrakeResourceRootEnvironmentVariableName),
        *guess, resource_path);
  }

  // (2) Check the Runfiles. If and only if we have Drake's runfiles (not just
  // any old runfiles) should we consider runfiles as a source for FindResource.
  // Downstream projects that use Bazel but don't build Drake from source will
  // have runfiles but not Drake's runfiles; in that case we should skip this
  // option and continue to option (3) instead.
  if (HasRunfiles()) {
    if (FindRunfile(kSentinelRelpath).error.empty()) {
      return MakeResultFrom(resource_path, FindRunfile(resource_path));
    } else {
      log()->debug(
          "FindResource ignoring Bazel runfiles with no sentinel file {}.",
          kSentinelRelpath);
    }
  }

  // (3) Check the `libdrake_marker.so` location in the install tree.
  if (auto guess = MaybeGetInstallResourceRoot()) {
    return CheckAndMakeResult("Drake CMake install marker", *guess,
                              resource_path);
  }

  // No resource roots were found.
  return Result::make_error(
      resource_path,
      fmt::format(
          "Could not find Drake resource_path '{}' because no resource roots "
          "of any kind could be found: {} is unset, a {} could not be created "
          "or did not contain {}, and there is no Drake CMake install marker.",
          resource_path, kDrakeResourceRootEnvironmentVariableName,
          "bazel::tools::cpp::runfiles::Runfiles", kSentinelRelpath));
}

string FindResourceOrThrow(const string& resource_path) {
  return FindResource(resource_path).get_absolute_path_or_throw();
}

std::optional<string> ReadFile(const std::filesystem::path& path) {
  std::optional<string> result;
  std::ifstream input(path, std::ios::binary);
  if (input.is_open()) {
    std::stringstream content;
    content << input.rdbuf();
    result.emplace(std::move(content).str());
  }
  return result;
}

std::string ReadFileOrThrow(const std::filesystem::path& path) {
  std::optional<string> result = ReadFile(path);
  if (!result) {
    throw std::runtime_error(
        fmt::format("Error reading from '{}'", path.string()));
  }
  return std::move(*result);
}

}  // namespace drake
