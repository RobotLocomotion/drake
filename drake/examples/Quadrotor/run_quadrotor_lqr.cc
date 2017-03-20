/// @file
///
/// This demo sets up a controlled Quadrotor that uses a Linear Quadratic
/// Regulator to (locally) stabilize a nominal hover.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/Quadrotor/quadrotor_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_int32(simulation_trials, 10, "Number of trials to simulate.");
DEFINE_double(simulation_real_time_rate, 1.0, "Real time rate");
DEFINE_double(trial_duration, 7.0, "Duration of execution of each trial");

namespace drake {
using systems::DiagramBuilder;
using systems::Simulator;
using systems::Context;
using systems::ContinuousState;
using systems::VectorBase;

namespace examples {
namespace quadrotor {
namespace {

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      multibody::joints::kRollPitchYaw, tree.get());

  // The nominal hover position is at (0, 0, 1.0) in world coordinates.
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};

  auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();
  auto controller = builder.AddSystem(StabilizingLQRController(
      quadrotor, kNominalPosition));
  auto visualizer =
      builder.AddSystem<drake::systems::DrakeVisualizer>(*tree, &lcm);

  builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));
  builder.Connect(quadrotor->get_output_port(0), visualizer->get_input_port(0));

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);
  VectorX<double> x0 = VectorX<double>::Zero(12);

  const VectorX<double> kNominalState{((Eigen::VectorXd(12) << kNominalPosition,
  Eigen::VectorXd::Zero(9)).finished())};

  srand(42);

  for (int i = 0; i < FLAGS_simulation_trials; i++) {
    auto diagram_context = diagram->CreateDefaultContext();
    x0 = VectorX<double>::Random(12);

    simulator.get_mutable_context()
        ->get_mutable_continuous_state_vector()
        ->SetFromVector(x0);

    simulator.Initialize();
    simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);
    simulator.StepTo(FLAGS_trial_duration);

    // Goal state verification.
    const Context<double>& context = simulator.get_context();
    const ContinuousState<double>* state = context.get_continuous_state();
    const VectorX<double>& position_vector = state->CopyToVector();

    if (!is_approx_equal_abstol(
        position_vector, kNominalState, 1e-4)) {
      throw std::runtime_error("Target state is not achieved.");
    }

    simulator.reset_context(std::move(diagram_context));
  }
  return 0;
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::quadrotor::do_main(argc, argv);
}
