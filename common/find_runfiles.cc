#include "drake/common/find_runfiles.h"

#include <cstdlib>
#include <memory>
#include <stdexcept>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include "fmt/format.h"
#include "tools/cpp/runfiles/runfiles.h"

#include "drake/common/drake_assert.h"
#include "drake/common/filesystem.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

using bazel::tools::cpp::runfiles::Runfiles;

namespace drake {
namespace {

// Replace `nullptr` with `"nullptr",` or else just return `arg` unchanged.
const char* nullable_to_string(const char* arg) {
  return arg ? arg : "nullptr";
}

// Either a bazel_tools Runfiles object xor an error string.
struct RunfilesSingleton {
  std::unique_ptr<Runfiles> runfiles;
  std::string runfiles_dir;
  std::string error;
};

// Create a bazel_tools Runfiles object xor an error string.  This is memoized
// by GetSingletonRunfiles (i.e., this is only called once per process).
RunfilesSingleton Create() {
  const char* mechanism{};
  RunfilesSingleton result;
  std::string bazel_error;

  // Chose a mechanism based on environment variables.
  if (std::getenv("TEST_SRCDIR")) {
    // When running under bazel test, use the test heuristics.
    mechanism = "TEST_SRCDIR";
    result.runfiles.reset(Runfiles::CreateForTest(&bazel_error));
  } else if ((std::getenv("RUNFILES_MANIFEST_FILE") != nullptr) ||
             (std::getenv("RUNFILES_DIR") != nullptr)) {
    // When running with some RUNFILES_* env already set, just use that.
    mechanism = "RUNFILES_{MANIFEST_FILE,DIR}";
    result.runfiles.reset(Runfiles::Create({}, &bazel_error));
  } else {
    // When running from the user's command line, use argv0.
    mechanism = "argv0";
#ifdef __APPLE__
    std::string argv0;
    argv0.resize(4096);
    uint32_t buf_size = argv0.size();
    int error = _NSGetExecutablePath(&argv0.front(), &buf_size);
    if (error) {
      throw std::runtime_error("Error from _NSGetExecutablePath");
    }
    argv0 = argv0.c_str();  // Remove trailing nil bytes.
    drake::log()->debug("_NSGetExecutablePath = {}", argv0);
#else
    const std::string& argv0 = filesystem::read_symlink({
        "/proc/self/exe"}).string();
    drake::log()->debug("readlink(/proc/self/exe) = {}", argv0);
#endif
    result.runfiles.reset(Runfiles::Create(argv0, &bazel_error));
  }
  drake::log()->debug("FindRunfile mechanism = {}", mechanism);

  // If there were runfiles, identify the RUNFILES_DIR.
  if (result.runfiles) {
    for (const auto& key_value : result.runfiles->EnvVars()) {
      if (key_value.first == "RUNFILES_DIR") {
        // N.B. We must normalize the path; otherwise the path may include
        // `parent/./path` if the binary was run using `./bazel-bin/target` vs
        // `bazel-bin/target`.
        // TODO(eric.cousineau): Show this in Drake itself. This behavior was
        // encountered in Anzu issue 5653, in a Python binary.
        result.runfiles_dir =
            filesystem::path(key_value.second).lexically_normal().string();
        break;
      }
    }
    // If we didn't find it, something was very wrong.
    if (result.runfiles_dir.empty()) {
      bazel_error = "RUNFILES_DIR was not provided by the Runfiles object";
      result.runfiles.reset();
    } else if (!filesystem::is_directory({result.runfiles_dir})) {
      bazel_error = fmt::format(
          "RUNFILES_DIR '{}' does not exist", result.runfiles_dir);
      result.runfiles.reset();
    }
  }

  // Report any error.
  if (!result.runfiles) {
    result.runfiles_dir.clear();
    result.error = fmt::format(
        "{} (created using {} with TEST_SRCDIR={} and "
            "RUNFILES_MANIFEST_FILE={} and RUNFILES_DIR={})",
        bazel_error, mechanism,
        nullable_to_string(std::getenv("TEST_SRCDIR")),
        nullable_to_string(std::getenv("RUNFILES_MANIFEST_FILE")),
        nullable_to_string(std::getenv("RUNFILES_DIR")));
    drake::log()->debug("FindRunfile error: {}", result.error);
  }

  // Sanity check our return value.
  if (result.runfiles.get() == nullptr) {
    DRAKE_DEMAND(result.runfiles_dir.empty());
    DRAKE_DEMAND(result.error.length() > 0);
  } else {
    DRAKE_DEMAND(result.runfiles_dir.length() > 0);
    DRAKE_DEMAND(result.error.empty());
  }

  return result;
}

// Returns the RunfilesSingleton for the current process, latch-initializing it
// first if necessary.
const RunfilesSingleton& GetRunfilesSingleton() {
  static const never_destroyed<RunfilesSingleton> result{Create()};
  return result.access();
}

}  // namespace

bool HasRunfiles() {
  return GetRunfilesSingleton().runfiles.get() != nullptr;
}

RlocationOrError FindRunfile(const std::string& resource_path) {
  const auto& singleton = GetRunfilesSingleton();

  // Check for HasRunfiles.
  RlocationOrError result;
  if (!singleton.runfiles) {
    DRAKE_DEMAND(!singleton.error.empty());
    result.error = singleton.error;
    return result;
  }

  // Check the user input.
  if (resource_path.empty()) {
    result.error = "Resource path must not be empty";
    return result;
  }
  if (resource_path[0] == '/') {
    result.error = fmt::format(
        "Resource path '{}' must not be an absolute path", resource_path);
    return result;
  }

  // Locate the file on the manifest and in the directory.
  const std::string by_man = singleton.runfiles->Rlocation(resource_path);
  const std::string by_dir = singleton.runfiles_dir + "/" + resource_path;
  const bool by_man_ok = filesystem::is_regular_file({by_man});
  const bool by_dir_ok = filesystem::is_regular_file({by_dir});
  drake::log()->debug(
      "FindRunfile found by-manifest '{}' ({}) and by-directory '{}' ({})",
      by_man, by_man_ok ? "good" : "bad", by_dir, by_dir_ok ? "good" : "bad");

  if (by_man_ok && by_dir_ok) {
    // We must always return the directory-based result (not the manifest
    // result) because the result itself may be a file that contains embedded
    // relative pathnames.  The manifest result might actually be in the source
    // tree, not the runfiles directory, and in that case relative paths may
    // not work (e.g., when the relative path refers to a genfile).
    result.abspath = by_dir;
    return result;
  }

  // Report an error.
  const char* detail{};
  if (!by_man_ok && !by_dir_ok) {
    detail =
        "but the file does not exist at that location "
        "nor is it on the manifest";
  } else if (!by_man_ok && by_dir_ok) {
    detail =
        "and the file exists at that location "
        "but it is not on the manifest";
  } else {
    DRAKE_DEMAND(by_man_ok && !by_dir_ok);
    detail =
        "and it is on the manifest"
        "but the file does not exist at that location";
  }
  result.error = fmt::format(
      "Sought '{}' in runfiles directory '{}' {}; "
      "perhaps a 'data = []' dependency is missing.",
      resource_path, singleton.runfiles_dir, detail);
  return result;
}

}  // namespace drake
