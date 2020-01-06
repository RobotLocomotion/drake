#include "drake/common/scope_exit.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(ScopeExitTest, Example) {
  void* foo = ::malloc(10);
  ScopeExit guard([foo]() {
    ::free(foo);
  });
}

GTEST_TEST(ScopeExitTest, CountTest) {
  int count = 0;
  {
    ScopeExit guard([&count]() { ++count; });
  }
  EXPECT_EQ(count, 1);
}

GTEST_TEST(ScopeExitTest, DisarmTest) {
  int count = 0;
  {
    ScopeExit guard([&count]() { ++count; });
    guard.Disarm();
  }
  EXPECT_EQ(count, 0);
}

}  // namespace
}  // namespace drake
