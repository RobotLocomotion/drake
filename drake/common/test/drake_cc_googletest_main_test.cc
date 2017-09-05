/// @file
/// This is the device under test for drake_cc_googletest_main_test.py.
/// It is also run as a drake_cc_googletest in our BUILD file, for completeness.

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/text_logging.h"

// We expect the invoking shell to set this to 1.0 via the command-line.
DEFINE_double(magic_number, 0.0, "Magic number");

namespace drake {
namespace {

GTEST_TEST(GtestMainTest, BasicTest) {
  drake::log()->debug("Cross your fingers for the magic_number {}", 1.0);
  EXPECT_EQ(FLAGS_magic_number, 1.0);
}

}  // namespace
}  // namespace drake
