/// @file
/// This is a sample unit test whose purpose is to provide a binary for
/// test/drake_cc_googletest_main_test.py to run, in order to assess the
/// correctness of command-line flag handling.  That python file contains the
/// majority of the meaningful unit test logic.

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/text_logging.h"

// We expect the invoking shell to set this to 1.0 via the command-line.
// (If it doesn't, the unit test will fail.)
DEFINE_double(magic_number, 0.0, "Magic number");

namespace drake {
namespace {

GTEST_TEST(GtestMainTest, BasicTest) {
  drake::log()->debug("Cross your fingers for the magic_number {}", 1.0);
  EXPECT_EQ(FLAGS_magic_number, 1.0);
}

}  // namespace
}  // namespace drake
