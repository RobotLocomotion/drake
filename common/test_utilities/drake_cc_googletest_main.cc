/// @file
/// This is Drake's default main() function for gtest-based unit tests.

#include <cstdlib>
#include <fstream>

#include <gflags/gflags.h>
#include <gmock/gmock.h>

int main(int argc, char** argv) {
  std::cout << "Using drake_cc_googletest_main.cc\n";

  // Affirm to the test environment that we are cognizant of sharding.
  const char* const shard_ack_file = std::getenv("TEST_SHARD_STATUS_FILE");
  if (shard_ack_file != nullptr) {
    // Touch the file.
    std::ofstream my_file(shard_ack_file);
  }

  // Initialize gtest and gmock.
  testing::InitGoogleMock(&argc, argv);
  std::cout << "\n";  // Put a linebreak between gtest help and gflags help.

  // Initialize gflags; this must happen after gtest initialization, so that the
  // gtest flags have already been removed from argv and won't confuse gflags.
  google::SetUsageMessage(" ");  // Nerf a silly warning emitted by gflags.
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Actually run the tests.
  return RUN_ALL_TESTS();
}
