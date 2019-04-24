#include "drake/systems/framework/leaf_system.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace systems {
namespace leaf_system_detail {

// C++'s deprecation attribute only triggers a diagnostic for callers, not for
// overriders.  So our "DoHasDirectFeedthroughDeprecated" deprecation would
// typically never make it into users' hands.  In order to still make sure that
// they are notified, we'll log this warning at most once per process, if at
// runtime anyone overrides that method to return non-nullopt.  Drake itself
// never overrides the method, so hopefully this should never false-trigger.
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
