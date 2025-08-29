#include "drake/common/overloaded.h"

#include <string>
#include <variant>

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(OverloadedTest, CommentExampleTest) {
  // Test the exact text of the example in the file comment.
  using MyVariant = std::variant<int, std::string>;
  MyVariant v = 5;
  std::string result = std::visit<const char*>(  // BR
      overloaded{[](const int arg) {
                   return "found an int";
                 },
                 [](const std::string& arg) {
                   return "found a string";
                 }},
      v);
  EXPECT_EQ(result, "found an int");
}

GTEST_TEST(OverloadedTest, AutoTest) {
  using MyVariant = std::variant<int, std::string>;
  MyVariant v = 5;

  // An 'auto' arm doesn't match if there's any explicit match,
  // no matter if it's earlier or later in the list.
  std::string result = std::visit<const char*>(  // BR
      overloaded{
          [](const auto arg) {
            return "found an auto";
          },
          [](const int arg) {
            return "found an int";
          },
          [](const std::string& arg) {
            return "found a string";
          },
          [](const auto arg) {
            return "found an auto";
          },
      },
      v);
  EXPECT_EQ(result, "found an int");

  // An 'auto' arm matches if there's no explicit match.
  result = std::visit<const char*>(
      // BR
      overloaded{
          [](const auto arg) {
            return "found an auto";
          },
          [](const std::string& arg) {
            return "found a string";
          },
      },
      v);
  EXPECT_EQ(result, "found an auto");
}

}  // namespace
}  // namespace drake
