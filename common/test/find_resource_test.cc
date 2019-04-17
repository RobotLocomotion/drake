#include "drake/common/find_resource.h"

#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/drake_throw.h"

using std::string;

namespace drake {
namespace {

using Result = FindResourceResult;

GTEST_TEST(FindResourceTest, EmptyResult) {
  const auto& result = Result::make_empty();
  EXPECT_EQ(result.get_resource_path(), "");
  EXPECT_FALSE(result.get_absolute_path());
  EXPECT_THROW(result.get_absolute_path_or_throw(), std::runtime_error);
  ASSERT_TRUE(result.get_error_message());
}

GTEST_TEST(FindResourceTest, NonRelativeRequest) {
  const string abspath = "/dev/null";
  const auto& result = FindResource(abspath);
  EXPECT_EQ(result.get_resource_path(), abspath);

  // We don't get a path back.
  EXPECT_FALSE(result.get_absolute_path());
  EXPECT_THROW(result.get_absolute_path_or_throw(), std::runtime_error);

  // We get an error back.
  const optional<string> error_message = result.get_error_message();
  ASSERT_TRUE(error_message);
  EXPECT_EQ(*error_message, "resource_path is not a relative path");
}

GTEST_TEST(FindResourceTest, NotFound) {
  const string relpath = "drake/this_file_does_not_exist";
  const auto& result = FindResource(relpath);
  EXPECT_EQ(result.get_resource_path(), relpath);

  // We don't get a path back.
  EXPECT_FALSE(result.get_absolute_path());
  EXPECT_THROW(result.get_absolute_path_or_throw(), std::runtime_error);

  // We get an error back.
  const optional<string> error_message = result.get_error_message();
  ASSERT_TRUE(error_message);
  EXPECT_EQ(*error_message, "could not find resource: " + relpath);

  // Sugar works the same way.
  EXPECT_THROW(FindResourceOrThrow(relpath), std::runtime_error);
}

GTEST_TEST(FindResourceTest, FoundDeclaredData) {
  const string relpath = "drake/common/test/find_resource_test_data.txt";
  const auto& result = FindResource(relpath);
  EXPECT_EQ(result.get_resource_path(), relpath);

  // We don't get an error back.
  EXPECT_FALSE(result.get_error_message());

  // We get a path back.
  string absolute_path;
  EXPECT_NO_THROW(absolute_path = result.get_absolute_path_or_throw());
  ASSERT_TRUE(result.get_absolute_path());
  EXPECT_EQ(*result.get_absolute_path(), absolute_path);

  // The path is the correct answer.
  ASSERT_FALSE(absolute_path.empty());
  EXPECT_EQ(absolute_path[0], '/');
  std::ifstream input(absolute_path, std::ios::binary);
  ASSERT_TRUE(input);
  std::stringstream buffer;
  buffer << input.rdbuf();
  EXPECT_EQ(
      buffer.str(),
      "Test data for drake/common/test/find_resource_test.cc.\n");

  // Sugar works the same way.
  EXPECT_EQ(FindResourceOrThrow(relpath), absolute_path);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Check that adding a relative resource path fails on purpose.
GTEST_TEST(FindResourceTest, RelativeResourcePathShouldFail) {
  // Test `AddResourceSearchPath()` with a relative path. It is expected to
  // fail.
  const std::string test_directory = "find_resource_test_scratch";
  EXPECT_THROW(AddResourceSearchPath(test_directory), std::runtime_error);
}
#pragma GCC diagnostic pop

optional<std::string> GetEnv(const std::string& name) {
  const char* result = ::getenv(name.c_str());
  if (!result) { return nullopt; }
  return std::string(result);
}

void SetEnv(const std::string& name, const optional<std::string>& value) {
  if (value) {
    const int result = ::setenv(name.c_str(), value->c_str(), 1);
    DRAKE_THROW_UNLESS(result == 0);
  } else {
    const int result = ::unsetenv(name.c_str());
    DRAKE_THROW_UNLESS(result == 0);
  }
}

// Make a scope exit guard -- an object that when destroyed runs `func`.
auto MakeGuard(std::function<void()> func) {
  // The shared_ptr deleter func is always invoked, even for nullptrs.
  // http://en.cppreference.com/w/cpp/memory/shared_ptr/%7Eshared_ptr
  return std::shared_ptr<void>(nullptr, [=](void*) { func(); });
}

GTEST_TEST(FindResourceTest, FindUsingTestSrcdir) {
  // Confirm that the resource is found normally.
  const string relpath = "drake/common/test/find_resource_test_data.txt";
  EXPECT_TRUE(FindResource(relpath).get_absolute_path());

  // If we defeat both (1) the "look in current working directory" heuristic
  // and (2) the "look in TEST_SRCDIR" heuristic, then we should no longer be
  // able to find the resource.

  // Change cwd to be "/", but put it back when this test case ends.
  const spruce::path original_cwd = spruce::dir::getcwd();
  auto restore_cwd_guard = MakeGuard([original_cwd]() {
      const bool restore_ok = spruce::dir::chdir(original_cwd);
      DRAKE_DEMAND(restore_ok);
    });
  const bool chdir_ok = spruce::dir::chdir(spruce::path("/"));
  ASSERT_TRUE(chdir_ok);

  // Unset TEST_SRCDIR for a moment, and confirm that FindResource now fails.
  const std::string original_test_srcdir = GetEnv("TEST_SRCDIR").value_or("");
  ASSERT_FALSE(original_test_srcdir.empty());
  {
    auto restore_test_srcdir_guard = MakeGuard([original_test_srcdir]() {
        SetEnv("TEST_SRCDIR", original_test_srcdir);
      });
    SetEnv("TEST_SRCDIR", nullopt);

    // Can't find the resource anymore.
    EXPECT_TRUE(FindResource(relpath).get_error_message());
  }

  // Having TEST_SRCDIR back again is enough to get us working.
  EXPECT_TRUE(FindResource(relpath).get_absolute_path());
}

GTEST_TEST(GetDrakePathTest, BasicTest) {
  // Just test that we find a path, without any exceptions.
  const auto& result = MaybeGetDrakePath();
  ASSERT_TRUE(result);
  EXPECT_GT(result->length(), 5);
}

GTEST_TEST(GetDrakePathTest, PathIncludesDrake) {
  // Tests that the path returned includes the root of drake.
  const auto& result = MaybeGetDrakePath();
  ASSERT_TRUE(result);
  const spruce::path expected(*result +
                              "/common/test/find_resource_test_data.txt");
  EXPECT_TRUE(expected.exists());
}

// Create an empty file with the given filename.
void Touch(const std::string& filename) {
  std::ofstream(filename.c_str(), std::ios::out).close();
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// NOTE: This test modifies the result of calls to GetDrakePath() and variants.
// However, it does *not* clean up the modifications. As such, it must run
// *last*. Relying on execution order being alphabetical, we make sure it is
// in the last test suite and the last test case (with the ZZZ_ prefix).
GTEST_TEST(ZZZ_FindResourceTest, ZZZ_AlternativeDirectory) {
  // Test `AddResourceSearchPath()` and `GetResourceSearchPaths()` by creating
  // an empty file in a scratch directory with a sentinel file. Bazel tests are
  // run in a scratch directory, so we don't need to remove anything manually.
  const std::string test_directory = spruce::dir::getcwd().getStr() +
                                     "/find_resource_test_scratch";
  const std::string candidate_filename = "drake/candidate.ext";
  spruce::dir::mkdir(test_directory);
  spruce::dir::mkdir(test_directory + "/drake");
  Touch(test_directory + "/drake/.drake-find_resource-sentinel");
  Touch(test_directory + "/" + candidate_filename);
  AddResourceSearchPath(test_directory);
  EXPECT_TRUE(!GetResourceSearchPaths().empty());
  EXPECT_EQ(GetResourceSearchPaths()[0], test_directory);
  EXPECT_NO_THROW(drake::FindResourceOrThrow(candidate_filename));
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace drake
