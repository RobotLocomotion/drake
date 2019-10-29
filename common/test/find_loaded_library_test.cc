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
  // Test whether or not `LoadedLibraryPath()` can find the path to a library
  // loaded by the process.
  std::optional<string> library_path = LoadedLibraryPath("lib_is_real.so");
  ASSERT_TRUE(library_path);
  EXPECT_EQ(library_path.value()[0], '/');
  library_path = LoadedLibraryPath("lib_not_real.so");
  EXPECT_FALSE(library_path);
}

}  // namespace
}  // namespace drake
