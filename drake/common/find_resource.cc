#include "drake/common/find_resource.h"

#include <cstdlib>
#include <vector>
#include <utility>

#include <spruce.hh>

#include "drake/common/drake_throw.h"

using std::string;

namespace drake {

using Result = FindResourceResult;

optional<string>
Result::get_absolute_path() const {
  // If the resource was not found, return nothing.
  if (!base_path_) { return nullopt; }

  // Otherwise, return the the full path (base + relative).
  DRAKE_ASSERT(error_message_ == nullopt);
  return *base_path_ + '/' + resource_path_;
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
    DRAKE_ASSERT(base_path_ == nullopt);
    return error_message_;
  }

  // If successful, return no-error.
  if (base_path_ != nullopt) {
    return nullopt;
  }

  // Both optionals are null; we are empty; return a default error message.
  DRAKE_ASSERT(resource_path_.empty());
  return string("No resource was requested (empty result)");
}

string Result::get_resource_path() const {
  return resource_path_;
}

Result Result::make_success(string resource_path, string base_path) {
  DRAKE_THROW_UNLESS(!resource_path.empty());
  DRAKE_THROW_UNLESS(!base_path.empty());

  Result result;
  result.resource_path_ = std::move(resource_path);
  result.base_path_ = std::move(base_path);
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
    DRAKE_DEMAND(base_path_ == nullopt);
    DRAKE_DEMAND(error_message_ == nullopt);
  } else {
    // For our "non-empty" state, we must have exactly one of success or error.
    DRAKE_DEMAND((base_path_ == nullopt) != (error_message_ == nullopt));
  }
  // When present, the base_path and error_message cannot be an empty string.
  DRAKE_DEMAND((base_path_ == nullopt) || !base_path_->empty());
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

bool is_relative_path(const string& path) {
  // TODO(jwnimmer-tri) Prevent .. escape?
  return !path.empty() && (path[0] != '/');
}

bool file_exists(const string& dirpath, const string& relpath) {
  DRAKE_ASSERT(is_relative_path(relpath));
  const spruce::path dir_query(dirpath);
  if (!dir_query.isDir()) { return false; }
  const spruce::path file_query(dir_query.getStr() + '/' + relpath);
  return file_query.exists();
}

optional<string> check_candidate_dir(const spruce::path& candidate_dir) {
  // If we found the sentinel, we win.
  spruce::path candidate_file = candidate_dir;
  candidate_file.append(".drake-resource-sentinel");
  if (candidate_file.isFile()) {
    return candidate_dir.getStr();
  }

  return nullopt;
}

optional<string> find_sentinel_dir() {
  spruce::path candidate_dir;
  candidate_dir.setAsCurrent();
  int num_attempts = 0;
  while (true) {
    DRAKE_THROW_UNLESS(num_attempts < 1000);  // Insanity fail-fast.
    ++num_attempts;

    // If we fall off the end of the world somehow, stop.
    if (!candidate_dir.isDir()) {
      return nullopt;
    }

    // If we found the sentinel, we win.
    optional<string> result = check_candidate_dir(candidate_dir);
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

Result FindResource(string resource_path) {
  // Check if resource_path is well-formed.
  if (!is_relative_path(resource_path)) {
    return Result::make_error(
        std::move(resource_path),
        "resource_path is not a relative path");
  }

  // Collect a list of (priority-ordered) directories to check.
  std::vector<optional<string>> candidate_dirs;

  // (1) Search the environment variable first; if it works, it should always
  // win.  TODO(jwnimmer-tri) Should we split on colons, making this a PATH?
  candidate_dirs.emplace_back(getenv_optional(
      kDrakeResourceRootEnvironmentVariableName));

  // (2) Search in cwd (and its parent, grandparent, etc.) to find Drake's
  // resource-root sentinel file.
  candidate_dirs.emplace_back(find_sentinel_dir());

  // TODO(jwnimmer-tri) Add more search heuristics for installed copies of
  // Drake resources.

  // See which (if any) candidate contains the requested resource.
  for (const auto& candidate_dir : candidate_dirs) {
    if (candidate_dir && file_exists(*candidate_dir, resource_path)) {
      return Result::make_success(std::move(resource_path), *candidate_dir);
    }
  }

  // Nothing found.
  return Result::make_error(
      std::move(resource_path),
      "could not find resource");
}

}  // namespace drake
