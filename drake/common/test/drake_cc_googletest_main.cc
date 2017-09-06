/// @file
/// This is Drake's default main() function for gtest-based unit tests.

#include <gmock/gmock.h>

#include "drake/common/text_logging_gflags.h"

int main(int argc, char** argv) {
  std::cout << "Using drake/test/drake_cc_googletest_main.cc\n";

  // Initialize gtest and gmock.
  testing::InitGoogleMock(&argc, argv);
  std::cout << "\n";  // Put a linebreak between gtest help and gflags help.

  // Initialize gflags; this must happen after gtest initialization, so that the
  // gtest flags have already been removed from argv and won't confuse gflags.
  google::SetUsageMessage(" ");  // Nerf a silly warning emitted by gflags.
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Adjust Drake's log setting per the gflags results.
  drake::logging::HandleSpdlogGflags();

  // Actually run the tests.
  return RUN_ALL_TESTS();
}
