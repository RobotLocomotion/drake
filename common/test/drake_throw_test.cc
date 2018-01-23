#include "drake/common/drake_throw.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

GTEST_TEST(DrakeThrowTest, BasicTest) {
  EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true));
  EXPECT_THROW(DRAKE_THROW_UNLESS(false), std::runtime_error);
}

}  // namespace
