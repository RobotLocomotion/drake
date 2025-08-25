#include "drake/common/test_utilities/maybe_pause_for_user.h"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace drake {
namespace common {

void MaybePauseForUser(std::string_view message) {
  bool is_test = (std::getenv("TEST_TMPDIR") != nullptr);
  bool is_invoked_by_bazel_run =
      (std::getenv("BUILD_WORKSPACE_DIRECTORY") != nullptr);
  if (is_test && is_invoked_by_bazel_run) {
    // Nothing good will happen here. The prompt may not appear, and the
    // program will hang, failing to notice user keyboard input.
    return;
  }
  if (!message.empty()) {
    std::cout << message << std::endl;
  }
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

}  // namespace common
}  // namespace drake
