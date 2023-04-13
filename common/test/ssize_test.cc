#include "drake/common/ssize.h"

#include <array>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace {

// We have to polyfill for std::ssize() for C++ < C++20.
GTEST_TEST(Ssize, BasicTest) {
  int c[] = {-5, 10, 15};
  EXPECT_EQ(ssize(c), 3);

  std::array<double, 4> a = {1.0, 2.0, 3.0, 4.0};
  EXPECT_EQ(ssize(a), 4);

  std::string s = "abcdefg";
  EXPECT_EQ(ssize(s), 7);

  std::vector<int> v = {3, 1, 4, 1, 5, 9};
  EXPECT_EQ(ssize(v), 6);
}

}  // namespace
}  // namespace drake
