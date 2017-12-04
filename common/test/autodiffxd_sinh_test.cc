#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Sinh) {
  CHECK_UNARY_FUNCTION(sinh, x, y, 0.1);
  CHECK_UNARY_FUNCTION(sinh, x, y, -0.1);
  CHECK_UNARY_FUNCTION(sinh, y, x, 0.1);
  CHECK_UNARY_FUNCTION(sinh, y, x, -0.1);
}

}  // namespace
}  // namespace test
}  // namespace drake
