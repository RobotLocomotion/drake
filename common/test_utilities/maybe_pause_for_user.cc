#include "drake/common/test_utilities/maybe_pause_for_user.h"

#include <iostream>
#include <limits>

namespace drake {
namespace common {

void MaybePauseForUser() {
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

}  // namespace common
}  // namespace drake


