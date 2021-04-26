#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

class ExamplePublishingSystem final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExamplePublishingSystem)

  ExamplePublishingSystem() {
    DeclarePerStepPublishEvent(
        &ExamplePublishingSystem::Update);
  }

 private:
  systems::EventStatus Update(const systems::Context<double>&) const {
    return systems::EventStatus::Succeeded();
  }
};

// Tests that heap allocations do not occur from Simulator and the systems
// framework for systems that do unrestricted updates and do not have continuous
// state.
GTEST_TEST(SimulatorLimitMallocTest,
           NoHeapAllocsInSimulatorForSystemsWithoutContinuousState) {
  // Build a Diagram containing the Example system so we can test both Diagrams
  // and LeafSystems at once.
  DiagramBuilder<double> builder;
  builder.AddSystem<ExamplePublishingSystem>();
  auto diagram = builder.Build();

  // Create a Simulator and use it to advance time until t=3.
  Simulator<double> simulator(*diagram);
  // Trigger first (and only allowable) heap allocation.
  simulator.Initialize();
  {
    // TODO(rpoyner-tri): whittle allocations down to 0.
    test::LimitMalloc heap_alloc_checker({.max_num_allocations = 16});
    simulator.AdvanceTo(1.0);
    simulator.AdvanceTo(2.0);
    simulator.AdvanceTo(3.0);
    simulator.AdvancePendingEvents();
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
