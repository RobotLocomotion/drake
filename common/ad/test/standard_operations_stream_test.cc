#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"

namespace drake {
namespace test {
namespace {

GTEST_TEST(StandardOperationsTest, Stream) {
  const ad::AutoDiff x{0.25, 10, 0};
  std::stringstream stream;
  stream << x;
  EXPECT_EQ(stream.str(), "0.25");
}

}  // namespace
}  // namespace test
}  // namespace drake
