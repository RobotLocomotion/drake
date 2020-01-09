#include "drake/common/scope_exit.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

// Reproduce the header file example as a test case.  There are no GTEST
// predicates here; if the memory is not freed, we'll rely on our memory
// checkers to complain.
GTEST_TEST(ScopeExitTest, Example) {
  void* foo = ::malloc(10);
  ScopeExit guard([foo]() {
    ::free(foo);
  });
}

// Test that the func() is invoked.
GTEST_TEST(ScopeExitTest, CountTest) {
  int count = 0;
  {
    ScopeExit guard([&count]() { ++count; });
  }
  EXPECT_EQ(count, 1);
}

// Test that the guard can be disarmed.
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
