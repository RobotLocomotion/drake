#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"

namespace drake {
namespace {

TEST_F(AutoDiffXdTest, Acos) {
  CHECK_UNARY_FUNCTION(acos, x, y, 0.1);
  CHECK_UNARY_FUNCTION(acos, x, y, -0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, 0.1);
  CHECK_UNARY_FUNCTION(acos, y, x, -0.1);
}

}  // namespace
}  // namespace drake
