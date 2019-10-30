#include "drake/common/find_resource.h"

#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/drake_throw.h"
#include "drake/common/filesystem.h"
#include "drake/common/test_utilities/expect_no_throw.h"

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
  const std::optional<string> error_message = result.get_error_message();
  ASSERT_TRUE(error_message);
  EXPECT_EQ(*error_message,
            "Drake resource_path '/dev/null' is not a relative path.");
}

GTEST_TEST(FindResourceTest, NotFound) {
  const string relpath = "drake/this_file_does_not_exist";
  const auto& result = FindResource(relpath);
  EXPECT_EQ(result.get_resource_path(), relpath);

  // We don't get a path back.
  EXPECT_FALSE(result.get_absolute_path());
  EXPECT_THROW(result.get_absolute_path_or_throw(), std::runtime_error);

  // We get an error back.
  const std::optional<string> error_message = result.get_error_message();
  ASSERT_TRUE(error_message);
  EXPECT_THAT(*error_message, testing::ContainsRegex(
      "Sought '" + relpath + "' in runfiles.*not exist.*on the manifest"));

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
  DRAKE_EXPECT_NO_THROW(absolute_path = result.get_absolute_path_or_throw());
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

GTEST_TEST(FindResourceTest, FoundDirectory) {
  // Looking up a directory (not file) should fail.
  const string relpath = "drake/common";
  const auto& result = FindResource(relpath);
  ASSERT_TRUE(result.get_error_message());
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
  const filesystem::path expected =
      filesystem::path(*result) /
      filesystem::path("common/test/find_resource_test_data.txt");
  EXPECT_TRUE(filesystem::exists(expected));
}

}  // namespace
}  // namespace drake
