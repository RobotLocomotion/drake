#include <sstream>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Postdecrement) {
  AutoDiffDut x{0.25, 3, 0};
  EXPECT_EQ((x--).value(), 0.25);
  EXPECT_EQ(x.value(), -0.75);
  EXPECT_EQ(x.derivatives()[0], 1.0);
}

TEST_F(StandardOperationsTest, Predecrement) {
  AutoDiffDut x{0.25, 3, 0};
  EXPECT_EQ((--x).value(), -0.75);
  EXPECT_EQ(x.value(), -0.75);
  EXPECT_EQ(x.derivatives()[0], 1.0);
}

}  // namespace
}  // namespace test
}  // namespace drake
