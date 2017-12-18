#include "drake/common/find_loaded_library.h"

#include <string>

#include <gtest/gtest.h>

using std::string;

namespace drake {
namespace {

GTEST_TEST(FindLibraryTest, Library) {
  // Test wether or not `LoadedLibraryPath()` can find the path to a library
  // loaded by the process.
  optional<string> library_path =
  // TODO(fbudin69500): Remove this platform-dependency.
#ifdef __APPLE__
    LoadedLibraryPath("libgflags.so");
#else
    LoadedLibraryPath("libexternal_Scom_Ugithub_Ugflags_Ugflags_Slibgflags.so");
#endif
  EXPECT_TRUE(library_path);
  library_path = LoadedLibraryPath("lib_not_real.so");
  EXPECT_FALSE(library_path);
}

}  // namespace
}  // namespace drake
