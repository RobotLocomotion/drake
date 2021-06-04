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
    DeclarePeriodicPublishEvent(1.0, 0.5, &EventfulSystem::Update);
    DeclarePeriodicDiscreteUpdateEvent(1.0, 0.5, &EventfulSystem::Update);
    DeclarePeriodicUnrestrictedUpdateEvent(1.0, 0.5, &EventfulSystem::Update);
    DeclarePerStepPublishEvent(&EventfulSystem::Update);
    DeclarePerStepDiscreteUpdateEvent(&EventfulSystem::Update);
    DeclarePerStepUnrestrictedUpdateEvent(&EventfulSystem::Update);

    // These events were not found to allocate at AdvanceTo(); they are
    // included for completeness.
    DeclareForcedPublishEvent(&EventfulSystem::Update);
    DeclareForcedDiscreteUpdateEvent(&EventfulSystem::Update);
    DeclareForcedUnrestrictedUpdateEvent(&EventfulSystem::Update);

    // It turns out that declaring an init event can actually *reduce* the
    // allocation count, by forcing earlier allocations in underlying storage
    // objects. See #14543 for discussion of this problem.
    // TODO(rpoyner-tri): expand testing to cover this problem.
    DeclareInitializationPublishEvent(&EventfulSystem::Update);
    DeclareInitializationDiscreteUpdateEvent(&EventfulSystem::Update);
    DeclareInitializationUnrestrictedUpdateEvent(&EventfulSystem::Update);
  }

 private:
  EventStatus Update(const Context<double>&) const {
    return EventStatus::Succeeded();
  }
  EventStatus Update(const Context<double>&, DiscreteValues<double>*) const {
    return EventStatus::Succeeded();
  }
  EventStatus Update(const Context<double>&, State<double>*) const {
    return EventStatus::Succeeded();
  }
};

// Tests that heap allocations do not occur from Simulator and the systems
// framework for systems that do various event updates and do not have
// continuous state.
// TODO(rpoyner-tri): add testing for witness functions.
GTEST_TEST(SimulatorLimitMallocTest,
           NoHeapAllocsInSimulatorForSystemsWithoutContinuousState) {
  // Build a Diagram containing the test system so we can test both Diagrams
  // and LeafSystems at once.
  DiagramBuilder<double> builder;
  builder.AddSystem<EventfulSystem>();
  auto diagram = builder.Build();

  // Create a Simulator and use it to advance time until t=3.
  Simulator<double> simulator(*diagram);
  // Actually cause forced-publish events to be issued.
  simulator.set_publish_every_time_step(true);
  // Trigger first (and only allowable) heap allocation.
  simulator.Initialize();
  {
    // As long as there are any allocations allowed here, there are still
    // defects to fix. The exact number doesn't much matter; it should be set
    // to the minimum possible at any given revision, to catch regressions. The
    // goal (see #14543) is for the simulator and framework to support
    // heap-free simulation after initialization, given careful system
    // construction.
    test::LimitMalloc heap_alloc_checker({.max_num_allocations = 0});
    simulator.AdvanceTo(1.0);
    simulator.AdvanceTo(2.0);
    simulator.AdvanceTo(3.0);
    simulator.AdvancePendingEvents();
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
