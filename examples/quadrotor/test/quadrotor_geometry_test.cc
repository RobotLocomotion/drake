#include "drake/examples/quadrotor/quadrotor_geometry.h"

#include <gtest/gtest.h>

#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

GTEST_TEST(QuadrotorGeometryTest, AcceptanceTest) {
  // Just make sure nothing faults out.
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<QuadrotorPlant>();
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  auto geom = QuadrotorGeometry::AddToBuilder(
      &builder, plant->get_output_port(0), scene_graph);
  auto diagram = builder.Build();
  ASSERT_NE(geom, nullptr);

  EXPECT_TRUE(geom->get_frame_id().is_valid());
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
