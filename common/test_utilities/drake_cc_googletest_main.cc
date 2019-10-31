/// @file
/// This is Drake's default main() function for gtest-based unit tests.

#include <gflags/gflags.h>
#include <gmock/gmock.h>

// TODO(SeanCurtis-TRI): Remove this when CLion bazel plug-in no longer executes
// its debugger with the --gunit_color flag. Most recent versions known to be
// an issue:
//    CLion: 2017.1.3
//    Bazel plugin: 2017.07.05.0.2
// Related issue: https://github.com/bazelbuild/intellij/issues/131
DEFINE_string(gunit_color, "", "");

int main(int argc, char** argv) {
  std::cout << "Using drake_cc_googletest_main.cc\n";

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
