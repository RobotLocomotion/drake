#include <cstdlib>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace solvers {
namespace {

// This test is deliberately not in mosek_solver_test.cc to avoid causing
// license issues during tests in that file.
GTEST_TEST(MoseklmLicenseFileTest, MoseklmLicenseFileUnset) {
  const char* moseklm_license_file = std::getenv("MOSEKLM_LICENSE_FILE");

  if (moseklm_license_file != nullptr) {
    const int unsetenv_result = ::unsetenv("MOSEKLM_LICENSE_FILE");
    ASSERT_EQ(0, unsetenv_result);
  }

  MathematicalProgram program;
  MosekSolver solver;

  EXPECT_THROW(solver.Solve(program), std::runtime_error);

  if (moseklm_license_file != nullptr) {
    const int setenv_result =
        ::setenv("MOSEKLM_LICENSE_FILE", moseklm_license_file, 1);
    ASSERT_EQ(0, setenv_result);
  }
}

}  // namespace
}  // namespace solvers
}  // namespace drake
