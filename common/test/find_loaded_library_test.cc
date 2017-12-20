#include "drake/common/find_loaded_library.h"

#include <string>

#include <gtest/gtest.h>

using std::string;

// @note gcc requires us to consume something from `lib_is_real` to link the
// library.
void lib_is_real_dummy_function();

namespace drake {
namespace {

GTEST_TEST(FindLibraryTest, Library) {
  // See above.
  lib_is_real_dummy_function();
  // Test wether or not `LoadedLibraryPath()` can find the path to a library
  // loaded by the process.
  optional<string> library_path = LoadedLibraryPath("lib_is_real.so");
  EXPECT_TRUE(library_path);
  library_path = LoadedLibraryPath("lib_not_real.so");
  EXPECT_FALSE(library_path);
}

}  // namespace
}  // namespace drake
