#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, ToString) {
  const AutoDiffDut x{0.25, 3, 0};
  EXPECT_EQ(fmt::to_string(x), "0.25");
}

}  // namespace
}  // namespace test
}  // namespace drake
