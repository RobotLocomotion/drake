#include "drake/common/drake_throw.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"

namespace {

GTEST_TEST(DrakeThrowTest, BasicTest) {
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true));
  EXPECT_THROW(DRAKE_THROW_UNLESS(false), std::runtime_error);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
GTEST_TEST(DrakeThrowTest, DeprecatedTest) {
  int foo{};
  int* foo_pointer = &foo;
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(foo_pointer));
}
#pragma GCC diagnostic pop

}  // namespace
