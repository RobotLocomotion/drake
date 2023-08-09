#pragma once

#include <iostream>
#include <limits>

namespace drake {
namespace geometry {

/** Pause execution for the user to examine visualization output, for use
 within bazel unit tests. This function will not pause execution when running
 as a bazel test, but when running as a command-line executable, it will enable
 users to see the visualization outputs. */
void MaybePauseForUser() {
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

}  // namespace geometry
}  // namespace drake


