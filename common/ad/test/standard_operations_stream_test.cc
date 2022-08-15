#include <sstream>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/ad/test/standard_operations_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(StandardOperationsTest, Stream) {
  const AutoDiffDut x{0.25, 3, 0};
  std::stringstream stream;
  stream << x;
  EXPECT_EQ(stream.str(), "0.25");
}

}  // namespace
}  // namespace test
}  // namespace drake
