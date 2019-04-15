#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/pendulum/discrete_pendulum_plant.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/examples/pendulum/trajectory_optimization_common.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::UniformGravityFieldElement;

namespace drake {
namespace examples {
namespace pendulum {

namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(use_dircol, true,
            "True to use direct collocation as the transcription method. "
            "False to use direct transcription. Default is true "
            "(direct collocation).");
DEFINE_bool(use_mbp, true,
            "True to instantiate a MultibodyPlant. False to"
            "instantiate a PendulumPlant. Default is true "
            "(MultibodyPlant).");

std::unique_ptr<MultibodyPlant<double>> BuildMBPlant(
    double time_step, geometry::SceneGraph<double>* scene_graph = nullptr) {
  const char* const urdf_path = "drake/examples/pendulum/Pendulum.urdf";
  auto pendulum = std::make_unique<MultibodyPlant<double>>(time_step);
  pendulum->AddForceElement<UniformGravityFieldElement>();
  Parser sparser(pendulum.get(), scene_graph);
  sparser.AddModelFromFile(FindResourceOrThrow(urdf_path));
  pendulum->WeldFrames(pendulum->world_frame(),
                       pendulum->GetFrameByName("base_part2"));

  pendulum->Finalize(scene_graph);
  pendulum->set_name("pendulum");

  return pendulum;
}

int DoMain() {
  drake::solvers::MathematicalProgramResult result;
  std::unique_ptr<systems::trajectory_optimization::MultipleShooting> prog;

  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();


  if (FLAGS_use_mbp) {  /* Instantiate a MultibodyPlant */
    // DirectCollocation uses a continuous time plant. DirectTranscription
    // uses a discrete time plant.
    auto pendulum = FLAGS_use_dircol ? BuildMBPlant(0 /* time step */)
                                     : BuildMBPlant(0.05 /* time step */);

    // Setup and solve the MathematicalProgram.
    prog = DoTrajectoryOptimization(pendulum.get(), FLAGS_use_dircol, &result);
    if (!prog) { return 1; }

    // Create a new (continuous) MultibodyPlant for simulation which registers
    // geometry.
    auto sim_pendulum = BuildMBPlant(0.0, scene_graph);  // Continuous time MBP.
    const geometry::SourceId source_id = sim_pendulum->get_source_id().value();
    SimulateTrajectory(FLAGS_target_realtime_rate, std::move(sim_pendulum),
                       *scene_graph, source_id, *prog, result, &builder);
  } else if (!FLAGS_use_mbp) {
    if (FLAGS_use_dircol) {  /* use direct collocation */
      // Instantiate a continuous time plant for DirectCollocation.
      auto pendulum = std::make_unique<PendulumPlant<double>>();

      // Setup and solve the MathematicalProgram.
      prog =
          DoTrajectoryOptimization(pendulum.get(), FLAGS_use_dircol, &result);
      if (!prog) { return 1; }
    } else {  /* use direct transcription */
      // Instantiate a discrete time plant for DirectTranscription.
      auto pendulum =
          std::make_unique<DiscretePendulumPlant<double>>(0.05 /* time step */);

      // Setup and solve the MathematicalProgram.
      prog =
          DoTrajectoryOptimization(pendulum.get(), FLAGS_use_dircol, &result);
      if (!prog) { return 1; }
    }
    // Create a new (continuous) PendulumPlant for simulation which registers
    // geometry.
    auto sim_pendulum = std::make_unique<PendulumPlant<double>>();
    sim_pendulum->RegisterGeometry(
        sim_pendulum->get_parameters(*sim_pendulum->CreateDefaultContext()),
        scene_graph);
    const geometry::SourceId source_id = sim_pendulum->source_id();
    SimulateTrajectory(FLAGS_target_realtime_rate, std::move(sim_pendulum),
                       *scene_graph, source_id, *prog, result, &builder);
  }
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pendulum::DoMain();
}
