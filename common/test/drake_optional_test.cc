#include "drake/common/drake_optional.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(OptionalTest, BasicTest) {
  optional<int> foo;
  EXPECT_EQ(!!foo, false);

  foo = 1;
  ASSERT_EQ(!!foo, true);
  EXPECT_EQ(*foo, 1);
}

}  // namespace
}  // namespace drake
