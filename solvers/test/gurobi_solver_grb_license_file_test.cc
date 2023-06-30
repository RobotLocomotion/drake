#include <cstdlib>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

// These tests are deliberately not in gurobi_solver_test.cc to avoid causing
// license issues during tests in that file.

std::optional<std::string> GetEnvStr(const char* name) {
  const char* const value = std::getenv(name);
  if (!name) {
    return std::nullopt;
  }
  return std::string(value);
}

class GrbLicenseFileTest : public ::testing::Test {
 protected:
  GrbLicenseFileTest()
      : orig_grb_license_file_(GetEnvStr("GRB_LICENSE_FILE")) {}

  void SetUp() override {
    ASSERT_EQ(solver_.available(), true);
    ASSERT_TRUE(orig_grb_license_file_);

    // Add a variable to avoid the "Solve" function terminating without calling
    // the external Gurobi solver.
    prog_.NewContinuousVariables<1>();
  }

  void WriteTempLicenseFile(std::string contents) {
    std::string test_license = temp_directory() + "/test.lic";
    std::ofstream file(test_license);
    ASSERT_TRUE(file);
    file << contents;
    const int setenv_result =
        ::setenv("GRB_LICENSE_FILE", test_license.c_str(), 1);
    ASSERT_EQ(setenv_result, 0);
  }

  void TearDown() override {
    if (orig_grb_license_file_) {
      const int setenv_result =
          ::setenv("GRB_LICENSE_FILE", orig_grb_license_file_->c_str(), 1);
      EXPECT_EQ(setenv_result, 0);
    }
  }

  const std::optional<std::string> orig_grb_license_file_;
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
      solver_.Solve(prog_),
      ".*GurobiSolver has not been properly configured.*");
}


TEST_F(GrbLicenseFileTest, LocalLicenseFile) {
  EXPECT_EQ(solver_.enabled(), true);
  WriteTempLicenseFile(
      "license file contents that contains the string HOSTID and perhaps some "
      "other info.");
  solver_.Solve(prog_);
  EXPECT_TRUE(solver_.has_acquired_local_license());
}

TEST_F(GrbLicenseFileTest, ServerLicenseFile) {
  EXPECT_EQ(solver_.enabled(), true);
  WriteTempLicenseFile(
      "license file contents without the magic keyword.");
  solver_.Solve(prog_);
  EXPECT_FALSE(solver_.has_acquired_local_license());
}

}  // namespace
}  // namespace solvers
}  // namespace drake
