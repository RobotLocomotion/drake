#include <iostream>

#include "drake/common/find_loaded_library.h"

int TestAbsolutePath(const char* path) {
  const auto& result = drake::LoadedLibraryPath(path);
  // Check that returned path is absolute.
  if ((*result)[0] != '/') {
    throw "LoadedLibraryPath returned a relative path: " + *result;
  }
  return 0;
}

// Test that verifies that the path returned by `drake::LoadedLibraryPath()`
// is absolute, even if the path to the library in the executable is
// relative.
int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " library" << std::endl;
    return 1;
  }
  // Call the function twice to make sure that there is no difference
  // between the first and the second time (more specifically that
  // calling `dirname()` doesn't lead to modifying the underlying data
  // buffer).
  TestAbsolutePath(argv[1]);
  TestAbsolutePath(argv[1]);
  return 0;
}

