#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/examples/pendulum/trajectory_optimization_common.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::UniformGravityFieldElement;

namespace drake {
namespace examples {
namespace pendulum {

using trajectories::PiecewisePolynomial;

namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(use_dircol, true,
              "Indicates whether the optimization uses DirectCollocation. If"
              "this is false, use DirectTranscription.");

std::unique_ptr<MultibodyPlant<double>> BuildMBPlant(
    double time_step, geometry::SceneGraph<double>* scene_graph) {
  const char* const urdf_path = "drake/examples/pendulum/Pendulum.urdf";
  auto pendulum = std::make_unique<MultibodyPlant<double>>(time_step);
  pendulum->AddForceElement<UniformGravityFieldElement>();
  Parser sparser(pendulum.get(), scene_graph);
  sparser.AddModelFromFile(FindResourceOrThrow(urdf_path));
  pendulum->WeldFrames(pendulum->world_frame(),
                       pendulum->GetFrameByName("base_part2"));

  pendulum->Finalize(scene_graph);
  pendulum->set_name("spendulum");

  return pendulum;
}

int DoMain() {
  // Declare the shared objects between DirectCollocation and
  // DirectTranscription.
  std::unique_ptr<systems::trajectory_optimization::MultipleShooting> prog;
  drake::solvers::MathematicalProgramResult result;
  std::unique_ptr<MultibodyPlant<double>> pendulum;

  if (FLAGS_use_dircol) {
    // DirectCollocation uses a continuous time plant.
    pendulum = BuildMBPlant(0 /* time step */, nullptr);
    auto context = pendulum->CreateDefaultContext();
    const int actuation_port_index =
        pendulum->get_actuation_input_port().get_index();

    const int kNumTimeSamples = 21;
    const double kMinimumTimeStep = 0.2;
    const double kMaximumTimeStep = 0.5;

    auto dircol =
        std::make_unique<systems::trajectory_optimization::DirectCollocation>(
            pendulum.get(), *context, kNumTimeSamples, kMinimumTimeStep,
            kMaximumTimeStep, actuation_port_index);

    dircol->AddEqualTimeIntervalsConstraints();
    AddSwingupConstraints(dircol.get());

    result = solvers::Solve(*dircol);
    prog = std::move(dircol);
  } else {
    // DirectTranscription uses a discrete time plant.
    pendulum = BuildMBPlant(0.05 /* time step */, nullptr);
    auto context = pendulum->CreateDefaultContext();
    const int actuation_port_index =
        pendulum->get_actuation_input_port().get_index();

    const int kNumTimeSamples = 100;

    auto dirtran =
        std::make_unique<systems::trajectory_optimization::DirectTranscription>(
            pendulum.get(), *context, kNumTimeSamples, actuation_port_index);

    AddSwingupConstraints(dirtran.get());

    result = solvers::Solve(*dirtran);
    prog = std::move(dirtran);
  }

  // Confirm the optimization succeeded.
  if (!ResultIsSuccess(result)) {
    return 1;
  }

  // Now, build and run the simulation using a continuous time plant.
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();

  // Create a new plant which registers geometry specifically for simulation.
  auto sim_pendulum = BuildMBPlant(0.0, scene_graph);  // Continuous time MBP.
  const geometry::SourceId source_id = sim_pendulum->get_source_id().value();
  SimulateTrajectory(FLAGS_target_realtime_rate, std::move(sim_pendulum),
                     *scene_graph, source_id, *prog, result, &builder);

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
