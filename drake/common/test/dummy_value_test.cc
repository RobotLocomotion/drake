#include "drake/common/dummy_value.h"

#include <cmath>

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(DummyValueTest, Double) {
  EXPECT_TRUE(std::isnan(dummy_value<double>::get()));
}

GTEST_TEST(DummyValueTest, Int) {
  EXPECT_NE(dummy_value<int>::get(), 0);
}

}  // namespace
}  // namespace drake
