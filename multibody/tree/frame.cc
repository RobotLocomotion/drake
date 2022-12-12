#include "drake/multibody/tree/frame.h"

#include "drake/common/identifier.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace internal {

std::string DeprecateWhenEmptyName(std::string name, std::string_view type) {
  if (name.empty()) {
    throw std::runtime_error(fmt::format(
        "The name parameter to the {} constructor is required.", type));
  }
  return name;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
