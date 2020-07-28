#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/** Publish initialization events.

This should be used when you have a system that has publish events at
initialization (e.g. visualization). This should be preferred instead of using
a throwaway Simulator (e.g. `Simulator(system, context.Clone()).Initialize()`).

@note This will not capture events like discrete or unrestricted updates at
t=0. In cases like this, you should use a Simulator class.
 */
void PublishInitializationEvents(
    const System<double>& system,
    const Context<double>& context);

}  // namespace systems
}  // namespace drake
