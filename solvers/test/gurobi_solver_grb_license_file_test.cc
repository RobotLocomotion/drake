#include <cstdlib>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

// These tests are deliberately not in gurobi_solver_test.cc to avoid causing
// license issues during tests in that file.

GTEST_TEST(GrbLicenseFileTest, GrbLicenseFileSet) {
  const char* grb_license_file = std::getenv("GRB_LICENSE_FILE");
  ASSERT_STRNE(nullptr, grb_license_file);

  MathematicalProgram program;
  // Add a variable to avoid the "Solve" function terminating without calling
  // the external Gurobi solver.
  const auto x = program.NewContinuousVariables<1>();
  GurobiSolver solver;

  MathematicalProgramResult result;
  DRAKE_EXPECT_NO_THROW(solver.Solve(program, {}, {}, &result));
}

GTEST_TEST(GrbLicenseFileTest, GrbLicenseFileUnset) {
  const char* grb_license_file = std::getenv("GRB_LICENSE_FILE");
  ASSERT_STRNE(nullptr, grb_license_file);

  const int unsetenv_result = ::unsetenv("GRB_LICENSE_FILE");
  ASSERT_EQ(0, unsetenv_result);

  MathematicalProgram program;
  // Add a variable to avoid the "Solve" function terminating without calling
  // the external Gurobi solver.
  const auto x = program.NewContinuousVariables<1>();
  GurobiSolver solver;

  try {
    MathematicalProgramResult result;
    solver.Solve(program, {}, {}, &result);
    ADD_FAILURE() << "Expected exception of type std::runtime_error.";
  } catch (const std::exception& err) {
    EXPECT_EQ(err.what(), std::string(
        "drake::solvers::GurobiSolver::is_enabled() is false; "
        "see its documentation for how to enable."));
  } catch (...) {
    ADD_FAILURE() << "Expected std::exception.";
  }

  const int setenv_result = ::setenv("GRB_LICENSE_FILE", grb_license_file, 1);
  ASSERT_EQ(0, setenv_result);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
