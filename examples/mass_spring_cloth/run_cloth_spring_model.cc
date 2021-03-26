/*
 @brief  A discrete mass-spring system example.
*/

#include <cstdlib>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/mass_spring_cloth/cloth_spring_model.h"
#include "drake/examples/mass_spring_cloth/cloth_spring_model_geometry.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_int32(nx, 20, "Number of particles in the x direction");
DEFINE_int32(ny, 20, "Number of particles in the y direction");
DEFINE_double(h, 0.05, "Separation between neighboring particles");
DEFINE_double(dt, 0.01,
              "Time step size for system. Discrete time stepping "
              "scheme described in Bridson et.al. will be used if dt > 0. The "
              "discrete scheme runs faster but is not error controlled. If dt "
              "<= 0, the system will be continuous");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "How long to simulate the system");

namespace drake {
namespace examples {
namespace mass_spring_cloth {
namespace {

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto* cloth_spring_model = builder.AddSystem<ClothSpringModel<double>>(
      FLAGS_nx, FLAGS_ny, FLAGS_h, FLAGS_dt);
  auto* scene_graph = builder.AddSystem<geometry::SceneGraph>();
  ClothSpringModelGeometry::AddToBuilder(&builder, *cloth_spring_model,
                                         scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(context));
  simulator->AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace
}  // namespace mass_spring_cloth
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "A simple demonstration of a cloth modeled as a mass-spring system");
  // Build the diagram for a visualized mass-spring system.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::mass_spring_cloth::DoMain();
}
