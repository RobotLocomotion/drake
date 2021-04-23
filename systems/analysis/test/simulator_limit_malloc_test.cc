#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

class EventfulSystem final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EventfulSystem)

  EventfulSystem() {
    // These events were found to cause allocations at AdvanceTo() as
    // originally implemented.
    DeclarePeriodicPublishEvent(0.7, 0.0, &EventfulSystem::Update);
    DeclarePerStepPublishEvent(&EventfulSystem::Update);

    // These events were not found to allocate at AdvanceTo(); they are
    // included for completeness.
    DeclareInitializationPublishEvent(&EventfulSystem::Update);
    DeclareForcedPublishEvent(&EventfulSystem::Update);
  }

 private:
  EventStatus Update(const Context<double>&) const {
    return EventStatus::Succeeded();
  }
};

// Tests that heap allocations do not occur from Simulator and the systems
// framework for systems that do various event updates and do not have
// continuous state.
GTEST_TEST(SimulatorLimitMallocTest,
           NoHeapAllocsInSimulatorForSystemsWithoutContinuousState) {
  // Build a Diagram containing the test system so we can test both Diagrams
  // and LeafSystems at once.
  DiagramBuilder<double> builder;
  builder.AddSystem<EventfulSystem>();
  auto diagram = builder.Build();

  // Create a Simulator and use it to advance time until t=3.
  Simulator<double> simulator(*diagram);
  // Trigger first (and only allowable) heap allocation.
  simulator.Initialize();
  {
    // TODO(rpoyner-tri): whittle allocations down to 0.
    test::LimitMalloc heap_alloc_checker({.max_num_allocations = 58});
    simulator.AdvanceTo(1.0);
    simulator.AdvanceTo(2.0);
    simulator.AdvanceTo(3.0);
    simulator.AdvancePendingEvents();
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
