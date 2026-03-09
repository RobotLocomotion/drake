#include "drake/common/find_cache.h"

#include <cstdlib>
#include <filesystem>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/set_env.h"

namespace drake {
namespace internal {
namespace {

using test::SetEnv;

namespace fs = std::filesystem;

// Check the TEST_TMPDIR case.
GTEST_TEST(FindCacheTest, TestTmpdir) {
  // Deny attempts to run this as `bazel-bin/...`; always use `bazel test`.
  DRAKE_DEMAND(std::getenv("TEST_TMPDIR") != nullptr);

  // Check the DUT.
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
