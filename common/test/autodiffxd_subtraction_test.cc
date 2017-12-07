#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Subtraction) {
  CHECK_BINARY_OP(-, x, y, 1.0);
  CHECK_BINARY_OP(-, x, y, -1.0);
  CHECK_BINARY_OP(-, y, x, 1.0);
  CHECK_BINARY_OP(-, y, x, -1.0);
}

}  // namespace
}  // namespace test
}  // namespace drake
