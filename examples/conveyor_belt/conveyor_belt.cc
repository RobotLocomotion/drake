#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/value.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace examples {
namespace conveyor_belt {
namespace {

DEFINE_double(max_time_step, 1e-2, "Simulation time step used for integrator.");
DEFINE_double(simulation_time, 100.0, "Duration of the simulation in seconds.");
DEFINE_double(realtime_rate, 1.0, "Target realtime rate.");
DEFINE_double(belt_speed, 0.5,
              "Surface speed (m/s) commanded for every body that declares a "
              "surface velocity axis.");

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_max_time_step);

  multibody::Parser(&builder).AddModelsFromUrl(
      "package://drake/examples/conveyor_belt/conveyor_example.dmd.yaml");

  plant.Finalize();

  // Command a constant surface speed for every body that declared a surface
  // velocity axis. Each signal on the "surface_speeds" bus is keyed by the
  // body's scoped name.
  {
    systems::BusValue speeds;
    for (multibody::BodyIndex i(0); i < plant.num_bodies(); ++i) {
      const multibody::RigidBody<double>& body = plant.get_body(i);
      if (plant.GetSurfaceVelocityAxis(body).has_value()) {
        speeds.Set(body.scoped_name().to_string(),
                   Value<double>(FLAGS_belt_speed));
      }
    }
    auto* speed_source =
        builder.AddSystem<systems::ConstantValueSource<double>>(
            Value<systems::BusValue>(speeds));
    builder.Connect(speed_source->get_output_port(),
                    plant.get_surface_speeds_input_port());
  }

  visualization::AddDefaultVisualization(&builder);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace conveyor_belt
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  // Initialize gflags.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::conveyor_belt::do_main();
}
