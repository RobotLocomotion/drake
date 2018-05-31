#include "drake/common/find_resource.h"

#include <cstdlib>
#include <utility>
#include <vector>

#include <spruce.hh>

#include "drake/common/drake_marker.h"
#include "drake/common/drake_throw.h"
#include "drake/common/find_loaded_library.h"
#include "drake/common/never_destroyed.h"

using std::string;

namespace drake {

using Result = FindResourceResult;

optional<string>
Result::get_absolute_path() const {
  return absolute_path_;
}

string
Result::get_absolute_path_or_throw() const {
  // If we have a path, return it.
  const auto& optional_path = get_absolute_path();
  if (optional_path) { return *optional_path; }

  // Otherwise, throw the error message.
  const auto& optional_error = get_error_message();
  DRAKE_ASSERT(optional_error != nullopt);
  throw std::runtime_error(*optional_error);
}

optional<string>
Result::get_error_message() const {
  // If an error has been set, return it.
  if (error_message_ != nullopt) {
    DRAKE_ASSERT(absolute_path_ == nullopt);
    return error_message_;
  }

  // If successful, return no-error.
  if (absolute_path_ != nullopt) {
    return nullopt;
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
    DRAKE_DEMAND(absolute_path_ == nullopt);
    DRAKE_DEMAND(error_message_ == nullopt);
  } else {
    // For our "non-empty" state, we must have exactly one of success or error.
    DRAKE_DEMAND((absolute_path_ == nullopt) != (error_message_ == nullopt));
  }
  // When non-nullopt, the path and error cannot be the empty string.
  DRAKE_DEMAND((absolute_path_ == nullopt) || !absolute_path_->empty());
  DRAKE_DEMAND((error_message_ == nullopt) || !error_message_->empty());
}

namespace {

optional<std::string> getenv_optional(const char* const name) {
  const char* const value = std::getenv(name);
  if (value) {
    return string(value);
  }
  return nullopt;
}

// If we are linked against libdrake_marker.so, return a candidate directory
// based on libdrake_marker.so's path; otherwise, return nullopt.  The
// resulting string will already end with "drake"; that is, the directory will
// contain files named like "common/foo.txt", not "drake/common/foo.txt".
optional<std::string>  GetCandidateDirFromLibDrakeMarker() {
  // Ensure that we have the library loaded.
  DRAKE_DEMAND(drake::internal::drake_marker_lib_check() == 1234);
  optional<std::string> libdrake_dir = LoadedLibraryPath("libdrake_marker.so");
  if (libdrake_dir) {
    libdrake_dir = libdrake_dir.value() + "/../share/drake";
  }
  return libdrake_dir;
}

// Returns true iff the path is relative (not absolute).
bool IsRelativePath(const string& path) {
  // TODO(jwnimmer-tri) Prevent .. escape?
  return !path.empty() && (path[0] != '/');
}

// Returns the absolute_path iff the `$dirpath/$relpath` exists, else nullopt.
// As a convenience to callers, if `dirpath` is nullopt, the result is nullopt.
// (To inquire about an empty `dirpath`, pass the empty string, not nullopt.)
optional<string> FileExists(
    const optional<string>& dirpath, const string& relpath) {
  DRAKE_ASSERT(IsRelativePath(relpath));
  if (!dirpath) { return nullopt; }
  const spruce::path dir_query(*dirpath);
  if (!dir_query.isDir()) { return nullopt; }
  const spruce::path file_query(dir_query.getStr() + '/' + relpath);
  if (!file_query.exists()) { return nullopt; }
  return file_query.getStr();
}

// Given a path like /foo/bar, returns the path /foo/bar/drake.
// Iff the path is nullopt, then the result is nullopt.
optional<string> AppendDrakeTo(const optional<string>& path) {
  if (path) {
    spruce::path result = *path;
    result.append("drake");
    return result.getStr();
  }
  return nullopt;
}

// Returns candidate_dir iff it exists and contains our sentinel file.
// Candidate paths will already end with "drake" as their final path element,
// or possibly a related name like "drake2"; that is, they will contain files
// named like "common/foo.txt", not "drake/common/foo.txt".
optional<string> CheckCandidateDir(const spruce::path& candidate_dir) {
  // If we found the sentinel, we win.
  spruce::path candidate_file = candidate_dir;
  candidate_file.append(".drake-find_resource-sentinel");
  if (candidate_file.isFile()) {
    return candidate_dir.getStr();
  }
  return nullopt;
}

// Returns a sentinel directory appropriate when running under `bazel test`.
optional<string> GetTestRunfilesDir() {
  // These environment variables are documented at:
  // https://docs.bazel.build/versions/master/test-encyclopedia.html#initial-conditions
  // We check TEST_TMPDIR as a sanity check that we're being called by Bazel.
  // Other than TEST_SRCDIR below, its the only other non-standard environment
  // variable that Bazel is required to set when running a test.
  if (::getenv("TEST_TMPDIR") == nullptr) {
    // Not running under `bazel test`.
    return nullopt;
  }
  char* test_srcdir = ::getenv("TEST_SRCDIR");
  if (test_srcdir == nullptr) {
    // Apparently running under `bazel test`, but no runfiles tree is set?
    // Maybe TEST_TMPDIR was something other than Bazel; ignore it.
    return nullopt;
  }
  return CheckCandidateDir(*AppendDrakeTo(string(test_srcdir)));
}

// Returns the directory that contains our sentinel file, searching from the
// current directory and working up through all transitive parent directories
// up to "/".  Candidate paths will already end with "drake" as their final
// path element, or possibly a related name like "drake2"; that is, they will
// contain files named like "common/foo.txt", not "drake/common/foo.txt".
optional<string> FindSentinelDir() {
  spruce::path candidate_dir = spruce::dir::getcwd();
  int num_attempts = 0;
  while (true) {
    DRAKE_THROW_UNLESS(num_attempts < 1000);  // Insanity fail-fast.
    ++num_attempts;

    // If we fall off the end of the world somehow, stop.
    if (!candidate_dir.isDir()) {
      return nullopt;
    }

    // If we found the sentinel, we win.
    optional<string> result = CheckCandidateDir(candidate_dir);
    if (result) {
      return result;
    }

    // Move up one directory; with spruce, "root" means "parent".
    candidate_dir = candidate_dir.root();
  }
}

}  // namespace

const char* const kDrakeResourceRootEnvironmentVariableName =
    "DRAKE_RESOURCE_ROOT";

// Saves search directorys path in a persistent variable.
// This function is only accessible from this file and should not
// be used outside of `GetResourceSearchPaths()` and
// `AddResourceSearchPath()`.
namespace {
std::vector<string>& GetMutableResourceSearchPaths() {
  static never_destroyed<std::vector<string>> search_paths;
  return search_paths.access();
}
}  // namespace

std::vector<string> GetResourceSearchPaths() {
  return GetMutableResourceSearchPaths();
}

void AddResourceSearchPath(string search_path) {
  // Throw an error if path is relative.
  DRAKE_THROW_UNLESS(!IsRelativePath(search_path));
  GetMutableResourceSearchPaths().push_back(std::move(search_path));
}

Result FindResource(string resource_path) {
  // Check if resource_path is well-formed: a relative path that starts with
  // "drake" as its first directory name.  A valid example would look like:
  // "drake/common/test/find_resource_test_data.txt".  Requiring strings passed
  // to drake::FindResource to start with "drake" is redundant, but preserves
  // compatibility with the original semantics of this function; if we want to
  // offer a function that takes paths without "drake", we can use a new name.
  if (!IsRelativePath(resource_path)) {
    return Result::make_error(
        std::move(resource_path),
        "resource_path is not a relative path");
  }
  const std::string prefix("drake/");
  if (resource_path.compare(0, prefix.size(), prefix) != 0) {
    return Result::make_error(
        std::move(resource_path),
        "resource_path does not start with " + prefix);
  }
  const std::string resource_path_substr = resource_path.substr(prefix.size());

  // Collect a list of (priority-ordered) directories to check.  Candidate
  // paths will already end with "drake" as their final path element, or
  // possibly a related name like "drake2"; that is, they will contain files
  // named like "common/foo.txt", not "drake/common/foo.txt".
  std::vector<optional<string>> candidate_dirs;

  // (1) Search the environment variable first; if it works, it should always
  // win.  TODO(jwnimmer-tri) Should we split on colons, making this a PATH?
  candidate_dirs.emplace_back(AppendDrakeTo(getenv_optional(
      kDrakeResourceRootEnvironmentVariableName)));

  // (2) Add the list of paths given programmatically. Paths are added only
  // if the sentinel file can be found.
  for (const auto& search_path : GetMutableResourceSearchPaths()) {
    spruce::path candidate_dir(*AppendDrakeTo(search_path));
    candidate_dirs.emplace_back(CheckCandidateDir(candidate_dir));
  }

  // (3) Find where `libdrake_marker.so` is, and add search path that
  // corresponds to resource folder in install tree based on
  // `libdrake_marker.so` location.
  candidate_dirs.emplace_back(GetCandidateDirFromLibDrakeMarker());

  // (4) Find resources during `bazel test` execution.
  candidate_dirs.emplace_back(GetTestRunfilesDir());

  // (5) Search in cwd (and its parent, grandparent, etc.) to find Drake's
  // resource-root sentinel file.
  candidate_dirs.emplace_back(FindSentinelDir());

  // Make sure that candidate_dirs are not relative paths. This could cause
  // bugs, but in theory it should never happen as the code above should
  // guard against it.
  for (const auto& candidate_dir : candidate_dirs) {
    if (candidate_dir && IsRelativePath(candidate_dir.value())) {
        string error_message = "path is not absolute: " + candidate_dir.value();
        return Result::make_error(std::move(resource_path), error_message);
      }
    }

  // See which (if any) candidate contains the requested resource.
  for (const auto& candidate_dir : candidate_dirs) {
    if (auto absolute_path = FileExists(candidate_dir, resource_path_substr)) {
      return Result::make_success(
          std::move(resource_path), std::move(*absolute_path));
    }
  }

  // Nothing found.
  string error_message = "could not find resource: " + resource_path;
  return Result::make_error(std::move(resource_path), error_message);
}

std::string FindResourceOrThrow(std::string resource_path) {
  return FindResource(std::move(resource_path)).get_absolute_path_or_throw();
}

}  // namespace drake
