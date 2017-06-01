#include "drake/common/find_resource.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

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
  const string relpath = "this file does not exist";
  const auto& result = FindResource(relpath);
  EXPECT_EQ(result.get_resource_path(), relpath);

  // We don't get a path back.
  EXPECT_FALSE(result.get_absolute_path());
  EXPECT_THROW(result.get_absolute_path_or_throw(), std::runtime_error);

  // We get an error back.
  const optional<string> error_message = result.get_error_message();
  ASSERT_TRUE(error_message);
  EXPECT_EQ(*error_message, "could not find resource");
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
}

}  // namespace
}  // namespace drake
