#include "drake/systems/framework/publish_initialization_events.h"

#include <memory>

#include "drake/systems/framework/event_collection.h"

namespace drake {
namespace systems {

void PublishInitializationEvents(
    const System<double>& system,
    const Context<double>& context) {
  std::unique_ptr<CompositeEventCollection<double>> events =
      system.AllocateCompositeEventCollection();
  system.GetInitializationEvents(context, events.get());
  system.Publish(context, events->get_publish_events());
}

}  // namespace systems
}  // namespace drake
