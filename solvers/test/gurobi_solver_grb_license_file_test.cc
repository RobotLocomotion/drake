#include <cstdlib>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

// These tests are deliberately not in gurobi_solver_test.cc to avoid causing
// license issues during tests in that file.

class GrbLicenseFileTest : public ::testing::Test {
 protected:
  GrbLicenseFileTest()
      : orig_grb_license_file_(std::getenv("GRB_LICENSE_FILE")) {}

  void SetUp() override {
    ASSERT_EQ(solver_.available(), true);
    ASSERT_STRNE(orig_grb_license_file_, nullptr);

    // Add a variable to avoid the "Solve" function terminating without calling
    // the external Gurobi solver.
    prog_.NewContinuousVariables<1>();
  }

  void TearDown() override {
    if (orig_grb_license_file_) {
      const int setenv_result = ::setenv(
          "GRB_LICENSE_FILE", orig_grb_license_file_, 1);
      EXPECT_EQ(setenv_result, 0);
    }
  }

  const char* const orig_grb_license_file_;
  MathematicalProgram prog_;
  GurobiSolver solver_;
};

TEST_F(GrbLicenseFileTest, GrbLicenseFileSet) {
  EXPECT_EQ(solver_.enabled(), true);
  DRAKE_EXPECT_NO_THROW(solver_.Solve(prog_));
}

TEST_F(GrbLicenseFileTest, GrbLicenseFileUnset) {
  EXPECT_EQ(solver_.enabled(), true);
  const int unsetenv_result = ::unsetenv("GRB_LICENSE_FILE");
  ASSERT_EQ(unsetenv_result, 0);
  EXPECT_EQ(solver_.enabled(), false);

  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_.Solve(prog_), std::exception,
      ".*GurobiSolver has not been properly configured.*");
}

}  // namespace
}  // namespace solvers
}  // namespace drake
