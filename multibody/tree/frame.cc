#include "drake/multibody/tree/frame.h"

#include "drake/common/identifier.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace internal {

// On 2022-12-01 when deprecation expires, this should change to throwing on
// empty names, instead of just logging.
std::string DeprecateWhenEmptyName(std::string name, std::string_view type) {
  if (!name.empty()) {
    return name;
  }
  static const logging::Warn log_once(
      "The name parameter to the {} constructor is now required. "
      "This will become a runtime exception on or after 2022-12-01.", type);
  return fmt::format("unnamed_{}_{}",
      type, drake::internal::get_new_identifier());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
