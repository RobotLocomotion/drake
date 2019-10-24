#include "drake/common/test_utilities/expect_no_throw.h"

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

namespace drake {
namespace {

void DoesNotThrow() { }

void DoesThrow() { throw std::runtime_error("Big time fail"); }

// Tests the ability for two identical matrices to be compared.
GTEST_TEST(NoThrowTest, AssertNoThrow) {
  DRAKE_ASSERT_NO_THROW(DoesNotThrow());
  EXPECT_THROW(
      DRAKE_ASSERT_NO_THROW(DoesThrow()), std::runtime_error);
}

GTEST_TEST(NoThrowTest, ExpectNoThrow) {
  DRAKE_EXPECT_NO_THROW(DoesNotThrow());
  EXPECT_NONFATAL_FAILURE(
      DRAKE_EXPECT_NO_THROW(DoesThrow()),
      R"""(Expected: Does not throw:
  DoesThrow()
Actual: Throws std::runtime_error
  Big time fail)""");
}

}  // namespace
}  // namespace drake
