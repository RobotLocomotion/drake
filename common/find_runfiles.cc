#include "drake/common/find_runfiles.h"

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <variant>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include <filesystem>

#include "tools/cpp/runfiles/runfiles.h"
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

using bazel::tools::cpp::runfiles::Runfiles;

namespace drake {
namespace {

namespace fs = std::filesystem;

// Replace `nullptr` with `"nullptr",` or else just return `arg` unchanged.
const char* nullable_to_string(const char* arg) {
  return arg ? arg : "nullptr";
}

// Runfiles details as loaded / provided by Bazel.
struct RunfilesSingleton {
  // This is the nominal, full-featured Runfiles; never nullptr.
  std::unique_ptr<Runfiles> runfiles;
  // This is the RUNFILES_DIR of `runfiles`; never empty.
  std::string runfiles_dir;
};

// Our singleton latch-initialization logic might fail. This type is akin to
// std::expected<RunfilesSingleton, ...> and is used to capture the result of
// initialization. Upon any failure, it will be the string error message instead
// of the singleton.
using RunfilesSingletonOrError =
    std::variant<std::string /* error */, RunfilesSingleton>;

// Create a bazel_tools Runfiles object xor an error string.  This is memoized
// by GetSingletonRunfiles (i.e., this is only called once per process).
RunfilesSingletonOrError Create() {
  RunfilesSingleton singleton;

  // Chose a mechanism based on environment variables.
  const char* mechanism{};
  std::string bazel_error;
  if (std::getenv("TEST_SRCDIR")) {
    // When running under bazel test, use the test heuristics.
    mechanism = "TEST_SRCDIR";
    singleton.runfiles.reset(Runfiles::CreateForTest(&bazel_error));
  } else if ((std::getenv("RUNFILES_MANIFEST_FILE") != nullptr) ||
             (std::getenv("RUNFILES_DIR") != nullptr)) {
    // When running with some RUNFILES_* env already set, just use that.
    mechanism = "RUNFILES_{MANIFEST_FILE,DIR}";
    singleton.runfiles.reset(Runfiles::Create({}, &bazel_error));
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
    const std::string& argv0 = fs::read_symlink({"/proc/self/exe"}).string();
    drake::log()->debug("readlink(/proc/self/exe) = {}", argv0);
#endif
    singleton.runfiles.reset(Runfiles::Create(argv0, &bazel_error));
  }
  drake::log()->debug("FindRunfile mechanism = {}", mechanism);
  drake::log()->debug("cwd = \"{}\"", fs::current_path().string());

  // We'll use a reusable helper function to provide a detailed error message.
  auto wrap_error = [&mechanism](const std::string& base_message) {
    std::string full_message = fmt::format(
        "{} (created using {} with TEST_SRCDIR={} and "
        "RUNFILES_MANIFEST_FILE={} and RUNFILES_DIR={})",
        base_message, mechanism, nullable_to_string(std::getenv("TEST_SRCDIR")),
        nullable_to_string(std::getenv("RUNFILES_MANIFEST_FILE")),
        nullable_to_string(std::getenv("RUNFILES_DIR")));
    drake::log()->debug("FindRunfile error: {}", full_message);
    return full_message;
  };

  // If there were no runfiles, error out.
  if (singleton.runfiles == nullptr) {
    return wrap_error(bazel_error);
  }

  // Identify the RUNFILES_DIR. Bazel always provides this in EnvVars.
  for (const auto& [key, value] : singleton.runfiles->EnvVars()) {
    if (key == "RUNFILES_DIR") {
      // N.B. We must normalize the path; otherwise the path may include
      // `parent/./path` if the binary was run using `./bazel-bin/target` vs
      // `bazel-bin/target`.
      // TODO(eric.cousineau): Show this in Drake itself. This behavior was
      // encountered in Anzu issue 5653, in a Python binary.
      fs::path path = value;
      path = fs::absolute(path);
      path = path.lexically_normal();
      if (!fs::is_directory(path)) {
        return wrap_error(fmt::format("RUNFILES_DIR {} does not exist",
                                      fmt_debug_string(path.string())));
      }
      singleton.runfiles_dir = path.string();
      break;
    }
  }
  if (singleton.runfiles_dir.empty()) {
    // This should be effectively unreachable code, but if something went
    // terribly wrong internally to Bazel, maybe it could happen.
    return wrap_error("RUNFILES_DIR was not provided by the Runfiles object");
  }

  // Sanity check our return value.
  DRAKE_DEMAND(singleton.runfiles != nullptr);
  DRAKE_DEMAND(!singleton.runfiles_dir.empty());

  return singleton;
}

// Returns the RunfilesSingleton for the current process, latch-initializing it
// first if necessary.
const RunfilesSingletonOrError& GetRunfilesSingletonOrError() {
  static const never_destroyed<RunfilesSingletonOrError> result{Create()};
  return result.access();
}

}  // namespace

bool HasRunfiles() {
  const RunfilesSingletonOrError& maybe = GetRunfilesSingletonOrError();
  return std::holds_alternative<RunfilesSingleton>(maybe);
}

RlocationOrError FindRunfile(const std::string& resource_path) {
  const RunfilesSingletonOrError& singleton_or_error =
      GetRunfilesSingletonOrError();

  // Check for HasRunfiles and grab the RunfilesSingleton (if it exists).
  RlocationOrError result;
  if (std::holds_alternative<std::string>(singleton_or_error)) {
    result.error = std::get<std::string>(singleton_or_error);
    return result;
  }
  const RunfilesSingleton& singleton =
      std::get<RunfilesSingleton>(singleton_or_error);

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
  const bool by_man_ok = fs::is_regular_file({by_man});
  const bool by_dir_ok = fs::is_regular_file({by_dir});
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
        "and it is on the manifest "
        "but the file does not exist at that location";
  }
  result.error = fmt::format(
      "Sought '{}' in runfiles directory '{}' {}; "
      "perhaps a 'data = []' dependency is missing.",
      resource_path, singleton.runfiles_dir, detail);
  return result;
}

}  // namespace drake
