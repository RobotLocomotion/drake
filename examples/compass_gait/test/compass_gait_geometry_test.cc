#include "drake/examples/compass_gait/compass_gait_geometry.h"

#include <gtest/gtest.h>

#include "drake/examples/compass_gait/compass_gait.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace compass_gait {
namespace {

GTEST_TEST(CompassGaitGeometryTest, AcceptanceTest) {
  // Just make sure nothing faults out.
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<CompassGait>();
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  auto geom = CompassGaitGeometry::AddToBuilder(
      &builder, plant->get_floating_base_state_output_port(), scene_graph);
  auto diagram = builder.Build();
  ASSERT_NE(geom, nullptr);
}

}  // namespace
}  // namespace compass_gait
}  // namespace examples
}  // namespace drake
