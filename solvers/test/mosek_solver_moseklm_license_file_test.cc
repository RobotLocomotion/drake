#include <cstdlib>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace solvers {
namespace {

// These tests is deliberately not in mosek_solver_test.cc to avoid causing
// license issues during tests in that file.

GTEST_TEST(MoseklmLicenseFileTest, MoseklmLicenseFileSet) {
  const char* moseklm_license_file = std::getenv("MOSEKLM_LICENSE_FILE");
  ASSERT_STRNE(nullptr, moseklm_license_file);

  MathematicalProgram program;
  // Add a variable to avoid the "Solve" function terminating without calling
  // the external MOSEK solver.
  const auto x = program.NewContinuousVariables<1>();
  MosekSolver solver;

  MathematicalProgramResult result;
  EXPECT_NO_THROW(solver.Solve(program, {}, {}, &result));
}

GTEST_TEST(MoseklmLicenseFileTest, MoseklmLicenseFileUnset) {
  const char* moseklm_license_file = std::getenv("MOSEKLM_LICENSE_FILE");
  ASSERT_STRNE(nullptr, moseklm_license_file);

  const int unsetenv_result = ::unsetenv("MOSEKLM_LICENSE_FILE");
  ASSERT_EQ(0, unsetenv_result);

  MathematicalProgram program;
  // Add a variable to avoid the "Solve" function terminating without calling
  // the external MOSEK solver.
  const auto x = program.NewContinuousVariables<1>();
  MosekSolver solver;

  try {
    MathematicalProgramResult result;
    solver.Solve(program, {}, {}, &result);
    ADD_FAILURE() << "Expected exception of type std::runtime_error.";
  } catch (const std::runtime_error& err) {
    EXPECT_EQ(err.what(), std::string("Could not locate MOSEK license file "
        "because MOSEKLM_LICENSE_FILE environment variable was not set."));
  } catch (...) {
    ADD_FAILURE() << "Expected exception of type std::runtime_error.";
  }

  const int setenv_result =
      ::setenv("MOSEKLM_LICENSE_FILE", moseklm_license_file, 1);
  ASSERT_EQ(0, setenv_result);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
