#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace {

GTEST_TEST(EigenAutodiffTypesTest, CheckingInheritance) {
  using ScalarLimits = std::numeric_limits<double>;
  using AutoDiffLimits = std::numeric_limits<AutoDiffXd>;
  const bool result = std::is_base_of_v<ScalarLimits, AutoDiffLimits>;
  EXPECT_TRUE(result);
}

}  // namespace
}  // namespace drake
