#include "drake/examples/acrobot/acrobot_geometry.h"

#include <gtest/gtest.h>

#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

GTEST_TEST(AcrobotGeometryTest, AcceptanceTest) {
  // Just make sure nothing faults out.
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<AcrobotPlant>();
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  auto geom = AcrobotGeometry::AddToBuilder(
      &builder, plant->get_output_port(0), scene_graph);
  auto diagram = builder.Build();
  ASSERT_NE(geom, nullptr);
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
