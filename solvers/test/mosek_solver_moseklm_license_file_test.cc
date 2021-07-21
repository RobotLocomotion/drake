#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace solvers {
namespace {

// These tests are deliberately not in mosek_solver_test.cc to avoid causing
// license issues during tests in that file.

std::optional<std::string> GetEnvStr(const char* name) {
  const char* const value = std::getenv(name);
  if (!name) {
    return std::nullopt;
  }
  return std::string(value);
}

class MoseklmLicenseFileTest : public ::testing::Test {
 protected:
  MoseklmLicenseFileTest()
      : orig_moseklm_license_file_(GetEnvStr("MOSEKLM_LICENSE_FILE")) {}

  void SetUp() override {
    ASSERT_EQ(solver_.available(), true);
    ASSERT_TRUE(orig_moseklm_license_file_);

    // Add a variable to avoid the "Solve" function terminating without calling
    // the external MOSEK solver.
    prog_.NewContinuousVariables<1>();
  }

  void TearDown() override {
    if (orig_moseklm_license_file_) {
      const int setenv_result = ::setenv(
          "MOSEKLM_LICENSE_FILE", orig_moseklm_license_file_->c_str(), 1);
      EXPECT_EQ(setenv_result, 0);
    }
  }

  const std::optional<std::string> orig_moseklm_license_file_;
  MathematicalProgram prog_;
  MosekSolver solver_;
};

TEST_F(MoseklmLicenseFileTest, MoseklmLicenseFileSet) {
  EXPECT_EQ(solver_.enabled(), true);
  DRAKE_EXPECT_NO_THROW(solver_.Solve(prog_));
}

TEST_F(MoseklmLicenseFileTest, MoseklmLicenseFileUnset) {
  EXPECT_EQ(solver_.enabled(), true);
  const int unsetenv_result = ::unsetenv("MOSEKLM_LICENSE_FILE");
  ASSERT_EQ(unsetenv_result, 0);
  EXPECT_EQ(solver_.enabled(), false);

  DRAKE_EXPECT_THROWS_MESSAGE(
      solver_.Solve(prog_),
      ".*MosekSolver has not been properly configured.*");
}

}  // namespace
}  // namespace solvers
}  // namespace drake
