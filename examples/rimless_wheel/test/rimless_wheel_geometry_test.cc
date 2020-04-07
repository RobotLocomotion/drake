#include "drake/examples/rimless_wheel/rimless_wheel_geometry.h"

#include <gtest/gtest.h>

#include "drake/examples/rimless_wheel/rimless_wheel.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace rimless_wheel {
namespace {

GTEST_TEST(RimlessWheelGeometryTest, AcceptanceTest) {
  // Just make sure nothing faults out.
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<RimlessWheel>();
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  auto geom = RimlessWheelGeometry::AddToBuilder(
      &builder, plant->get_floating_base_state_output_port(), scene_graph);
  auto diagram = builder.Build();
  ASSERT_NE(geom, nullptr);
}

}  // namespace
}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake
