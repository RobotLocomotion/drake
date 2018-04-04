#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Tanh) {
  CHECK_UNARY_FUNCTION(tanh, x, y, 0.1);
  CHECK_UNARY_FUNCTION(tanh, x, y, -0.1);
  CHECK_UNARY_FUNCTION(tanh, y, x, 0.1);
  CHECK_UNARY_FUNCTION(tanh, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
