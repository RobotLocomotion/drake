#include "drake/common/find_cache.h"

#include <cstdlib>
#include <filesystem>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"

namespace drake {
namespace internal {
namespace {

namespace fs = std::filesystem;

/* RAII to temporarily change an environment variable. */
class SetEnv {
 public:
  SetEnv(const std::string& var_name, std::optional<std::string> var_value)
      : var_name_(var_name) {
    const char* orig = std::getenv(var_name.c_str());
    if (orig != nullptr) {
      orig_value_ = orig;
    }
    if (var_value.has_value()) {
      ::setenv(var_name.c_str(), var_value->c_str(), 1);
    } else {
      ::unsetenv(var_name.c_str());
    }
  }

  ~SetEnv() {
    if (orig_value_.has_value()) {
      ::setenv(var_name_.c_str(), orig_value_->c_str(), 1);
    } else {
      ::unsetenv(var_name_.c_str());
    }
  }

 private:
  std::string var_name_;
  std::optional<std::string> orig_value_;
};

// Check the TEST_TMPDIR case.
GTEST_TEST(FindCacheTest, TestTmpdir) {
  const PathOrError result = FindOrCreateCache("foo");
  ASSERT_EQ(result.error, "");
  EXPECT_TRUE(fs::exists(result.abspath));
  const fs::path expected =
      fs::path(std::getenv("TEST_TMPDIR")) / ".cache" / "drake" / "foo";
  EXPECT_EQ(result.abspath, expected);
}

// Check the XDG_CACHE_HOME case.
GTEST_TEST(FindCacheTest, XdgCacheHome) {
  // Prepare the environment.
  const std::string xdg = temp_directory();
  const SetEnv env1("TEST_TMPDIR", std::nullopt);
  const SetEnv env2("XDG_CACHE_HOME", xdg);

  // Check the DUT.
  const PathOrError result = FindOrCreateCache("bar");
  ASSERT_EQ(result.error, "");
  EXPECT_TRUE(fs::exists(result.abspath));
  const fs::path expected = fs::path(xdg) / "drake" / "bar";
  EXPECT_EQ(result.abspath, expected);
}

// Check the HOME case.
GTEST_TEST(FindCacheTest, Home) {
  // Prepare the environment.
  const std::string home = temp_directory();
  const SetEnv env1("TEST_TMPDIR", std::nullopt);
  const SetEnv env2("USER", std::nullopt);
  const SetEnv env3("HOME", home);

  // Check the DUT.
  const PathOrError result = FindOrCreateCache("baz");
  ASSERT_EQ(result.error, "");
  EXPECT_TRUE(fs::exists(result.abspath));
  const fs::path expected = fs::path(home) / ".cache" / "drake" / "baz";
  EXPECT_EQ(result.abspath, expected);
}

// When the cache directory is read-only, we get a reasonable error.
GTEST_TEST(FindCacheTest, ReadOnlyCache) {
  // Prepare the environment.
  const std::string xdg = temp_directory();
  const SetEnv env1("TEST_TMPDIR", std::nullopt);
  const SetEnv env2("XDG_CACHE_HOME", xdg);

  // Remove all permissions.
  fs::permissions(xdg, fs::perms{});

  // Expect a failure.
  const PathOrError result = FindOrCreateCache("bar");
  EXPECT_EQ(result.abspath.string(), "");
  EXPECT_THAT(result.error, testing::MatchesRegex(".*not create.*drake.*"));
}

}  // namespace
}  // namespace internal
}  // namespace drake
