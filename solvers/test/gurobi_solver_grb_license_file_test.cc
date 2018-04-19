#include <cstdlib>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

// This test is deliberately not in gurobi_solver_test.cc to avoid causing
// license issues during tests in that file.
GTEST_TEST(GrbLicenseFileTest, GrbLicenseFileUnset) {
  const char* grb_license_file = std::getenv("GRB_LICENSE_FILE");

  if (grb_license_file != nullptr) {
    const int unsetenv_result = ::unsetenv("GRB_LICENSE_FILE");
    ASSERT_EQ(0, unsetenv_result);
  }

  MathematicalProgram program;
  GurobiSolver solver;

  EXPECT_THROW(solver.Solve(program), std::runtime_error);

  if (grb_license_file != nullptr) {
    const int setenv_result = ::setenv("GRB_LICENSE_FILE", grb_license_file, 1);
    ASSERT_EQ(0, setenv_result);
  }
}

}  // namespace
}  // namespace solvers
}  // namespace drake
