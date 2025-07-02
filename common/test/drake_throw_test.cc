#include "drake/common/drake_throw.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"

namespace {

GTEST_TEST(DrakeThrowTest, UnlessTest) {
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true));
  EXPECT_THROW(DRAKE_THROW_UNLESS(false), std::runtime_error);
}

GTEST_TEST(DrakeThrowTest, IfTest) {
  DRAKE_EXPECT_NO_THROW(DRAKE_THROW_IF(false));
  EXPECT_THROW(DRAKE_THROW_IF(true), std::runtime_error);
}

}  // namespace
