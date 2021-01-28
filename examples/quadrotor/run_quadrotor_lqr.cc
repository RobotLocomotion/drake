/// @file
///
/// This demo sets up a controlled Quadrotor that uses a Linear Quadratic
/// Regulator to (locally) stabilize a nominal hover.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/quadrotor/quadrotor_geometry.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
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

int do_main() {
  lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;

  // The nominal hover position is at (0, 0, 1.0) in world coordinates.
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};

  auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();
  quadrotor->set_name("quadrotor");
  auto controller = builder.AddSystem(StabilizingLQRController(
      quadrotor, kNominalPosition));
  controller->set_name("controller");
  builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
  builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));

  // Set up visualization
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  QuadrotorGeometry::AddToBuilder(
      &builder, quadrotor->get_output_port(0), scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph);

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
        .get_mutable_continuous_state_vector()
        .SetFromVector(x0);

    simulator.Initialize();
    simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);

    // The following accuracy is necessary for the example to satisfy its
    // ending state tolerances.
    simulator.get_mutable_integrator().set_target_accuracy(5e-5);
    simulator.AdvanceTo(FLAGS_trial_duration);

    // Goal state verification.
    const Context<double>& context = simulator.get_context();
    const ContinuousState<double>& state = context.get_continuous_state();
    const VectorX<double>& position_vector = state.CopyToVector();

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
  return drake::examples::quadrotor::do_main();
}
