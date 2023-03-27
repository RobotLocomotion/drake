#include "drake/common/find_cache.h"

#include <cstdlib>
#include <filesystem>
#include <optional>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace internal {
namespace {

namespace fs = std::filesystem;

#if defined(__APPLE__)
constexpr bool kApple = true;
#else
constexpr bool kApple = false;
#endif

/* If var_name is set, returns its value. Otherwise, returns nullopt. */
std::optional<std::string> GetStringEnv(const char* var_name) {
  const char* const var_value = std::getenv(var_name);
  if (var_value != nullptr) {
    return var_value;
  }
  return std::nullopt;
}

/* If var_name is set to a directory that exists, returns that path.
Otherwise, returns nullopt. */
std::optional<fs::path> GetPathEnv(const char* var_name) {
  const char* const var_value = std::getenv(var_name);
  if (var_value != nullptr) {
    auto path = fs::path{var_value};
    std::error_code ec;
    if (fs::is_directory(path, ec)) {
      return path;
    }
  }
  return std::nullopt;
}

/* If path exists, returns it. Otherwise, creates it and returns it. */
PathOrError CreateDirectory(fs::path path) {
  std::error_code ec;
  fs::create_directory(path, ec);
  if (ec) {
    return {.error = fmt::format("Could not create {}: {}", path.string(),
                                 ec.message())};
  }
  return {.abspath = fs::canonical(path)};
}

}  // namespace

PathOrError FindOrCreateCache(std::string_view subdir) {
  // We'll try the following options to find the ~/.cache, in order:
  // - ${TEST_TMPDIR}/.cache
  // - ${XDG_CACHE_HOME}
  // - /private/var/tmp/.cache_${USER}  (on Apple only)
  // - ${HOME}/.cache
  const std::optional<fs::path> test_tmpdir = GetPathEnv("TEST_TMPDIR");
  const std::optional<fs::path> xdg_cache_home = GetPathEnv("XDG_CACHE_HOME");
  const std::optional<std::string> user = GetStringEnv("USER");
  const std::optional<fs::path> home = GetPathEnv("HOME");
  PathOrError cache_dir;
  if (test_tmpdir.has_value()) {
    cache_dir = CreateDirectory(*test_tmpdir / ".cache");
  } else if (xdg_cache_home.has_value()) {
    cache_dir.abspath = *xdg_cache_home;
  } else if (kApple && user.has_value()) {
    cache_dir = CreateDirectory(fs::path("/private/var/tmp") /
                                fmt::format(".cache_{}", *user));
  } else if (home.has_value()) {
    cache_dir = CreateDirectory(*home / ".cache");
  } else {
    return {.error =
                "Could not determine an appropriate cache_dir to use. "
                "Set $XDG_CACHE_HOME to a valid scratch directory."};
  }
  if (!cache_dir.error.empty()) {
    return cache_dir;
  }

  // Create the Drake-specific subdirectory.
  auto cache_dir_drake = CreateDirectory(cache_dir.abspath / "drake");
  if (!cache_dir_drake.error.empty()) {
    return cache_dir_drake;
  }
  log()->debug("FindCache found {}", cache_dir_drake.abspath.string());

  // Create the requested subdirectory.
  return CreateDirectory(cache_dir_drake.abspath / subdir);
}

}  // namespace internal
}  // namespace drake
