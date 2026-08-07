/* @file
A four bar linkage demo demonstrating MultibodyPlant's automatic modeling of
closed kinematic loops, applied to a model parsed from an SDF file. It shows:
  - How to describe a four bar linkage in SDF as a plain loop of four joints,
    with no attempt to specify a spanning tree and no stand-in for the "extra"
    joint (see four_bar.sdf).
  - How to opt in to automatic loop modeling with
    `MultibodyPlant::SetEnableLoopTopology()`, and report what the resulting
    model looks like.
  - That there is nothing else to do. The loop is closed with a constraint that
    Finalize() adds, and because the model file describes the linkage in an
    assembled configuration that constraint is satisfied from the outset, so
    the mechanism can simply be simulated.

  Closing the loop takes a constraint, so this model asks for a solver that can
  enforce one, and it does not care which. It runs discrete under SAP, which is
  the default here, and continuous under CENIC:
      --time_step=0 --simulator_integration_scheme=cenic

  Compare the other two examples here, both of which model the same kind of
  mechanism:
  - four_bar_auto.cc builds an equivalent linkage through the C++ API rather
    than parsing it, which lets it start from an UNASSEMBLED configuration --
    something sdf and urdf cannot express, since they require only a single
    "joint frame" rather than Drake's more general model that says a joint
    connects an independent parent frame to an independent child. (The usd file
    format can express Drake's joint model.) That example assembles the linkage
    by simulating it, and goes to some trouble to draw the split link and the
    constraint that closes the loop, which are otherwise invisible.
  - four_bar_with_bushing.cc leaves one of the four joints out of its model file
    altogether and closes the loop with a LinearBushingRollPitchYaw force
    element instead of a constraint.

  Refer to README.md for more details.
*/
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {

using multibody::AddMultibodyPlantSceneGraph;
using multibody::Parser;
using systems::Context;
using systems::DiagramBuilder;
using systems::Simulator;

namespace examples {
namespace multibody {
namespace four_bar {
namespace {

DEFINE_double(simulation_time, 10.0, "Duration of the simulation in seconds.");

DEFINE_double(time_step, 1.0e-3,
              "Discrete time step of the MultibodyPlant in seconds, or 0 for a "
              "continuous plant. Closing the loop takes a constraint, so this "
              "needs a solver that supports one: discrete SAP, which is the "
              "default here, or the continuous CENIC integrator, via "
              "--time_step=0 --simulator_integration_scheme=cenic.");

DEFINE_double(applied_torque, 0.0,
              "Constant torque applied to the world_driver joint, in N·m.");

DEFINE_bool(interactive, true,
            "Show the linkage in the configuration the model file defines, and "
            "wait for you to press Enter before simulating. Set "
            "--nointeractive to run start to finish without stopping.");

int do_main() {
  // Build the MultibodyPlant and SceneGraph.
  DiagramBuilder<double> builder;
  auto [four_bar, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);

  // Opt in to automatic modeling of closed topologies. Without this, Finalize()
  // below would throw because of the loop in the model file.
  four_bar.SetEnableLoopTopology(true);

  Parser(&builder).AddModelsFromUrl(
      "package://drake/examples/multibody/four_bar/four_bar.sdf");

  // We are done defining the model.
  four_bar.Finalize();
  visualization::AddDefaultVisualization(&builder);
  auto diagram = builder.Build();

  // Create a context for the diagram and grab the four bar's sub-context.
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& four_bar_context =
      four_bar.GetMyMutableContextFromRoot(diagram_context.get());

  // A constant source for applied torque at the world_driver joint. This is the
  // only actuated joint in the model (the default is zero torque).
  four_bar.get_actuation_input_port().FixValue(&four_bar_context,
                                               FLAGS_applied_torque);

  // Note that we set no initial conditions; we're just at q=0 which is the
  // configuration the model file defines, which is already assembled.
  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  // Show the linkage as the model file defines it, before anything has moved,
  // and (unless --nointeractive) wait, so that there is time to open the
  // visualizer in a browser and have a look at it first.
  simulator->Initialize();
  diagram->ForcedPublish(simulator->get_context());
  fmt::print("\nThe linkage is shown as the model file defines it.\n");
  if (FLAGS_interactive) {
    std::cout << "Press Enter to simulate . . . " << std::flush;
    std::string line;
    std::getline(std::cin, line);  // Just eats the line; EOF is fine too.
  }
  // Restart the simulator's real time reference point. It throttles itself to
  // --simulator_target_realtime_rate (1, i.e. real time, by default) by
  // comparing simulated time against wall clock time elapsed since Initialize()
  // or ResetStatistics(); without this it would count the time we just spent
  // waiting as time it has to make up, and then run flat out to catch up.
  simulator->ResetStatistics();

  simulator->AdvanceTo(FLAGS_simulation_time);

  // Print some useful statistics.
  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace four_bar
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A four bar linkage demo demonstrating MultibodyPlant's automatic "
      "modeling of closed kinematic loops, from a model file. Launch meldis "
      "before running this example.");
  // Changes the default realtime rate to 1.0, so the visualization looks
  // realistic. Otherwise, it finishes so fast that we can't appreciate the
  // motion. Users can still change it on command-line, e.g. "
  // --simulator_target_realtime_rate=0.5" to slow it down.
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::four_bar::do_main();
}
