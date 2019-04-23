#include "drake/systems/framework/leaf_system.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace systems {
namespace leaf_system_detail {

void MaybeWarnDoHasDirectFeedthroughDeprecated() {
  static const logging::Warn log_once(
      "Overriding drake::systems::LeafSystem::DoHasDirectFeedthrough is "
      "deprecated; please consult its API documentation for alternatives.");
}

}  // namespace leaf_system_detail
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafSystem)
