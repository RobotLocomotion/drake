#include "drake/solvers/csdp_cpp_wrapper.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace solvers {
namespace internal {
// Try to solve a malformed program (k==0), expect an exception.
// This is a test for exit/setjmp/longjmp handling. See #16732.
GTEST_TEST(CsdpSolverInterrnalTest, ExitHandling) {
  csdp::blockmatrix C{};
  struct csdp::blockmatrix X, Z;
  double* y{};
  double pobj{}, dobj{};
  DRAKE_EXPECT_THROWS_MESSAGE(
      csdp::cpp_easy_sdp(nullptr, 0, 0, C, nullptr, nullptr, 0, &X, &y, &Z,
                         &pobj, &dobj),
      ".*CSDP.*fatal exception.*");
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
