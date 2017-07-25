#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/examples/pendulum/pendulum_swing_up.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::solvers::SolutionResult;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main() {
  systems::DiagramBuilder<double> builder;

  PendulumPlant<double>* pendulum{nullptr};
  systems::controllers::PidControlledSystem<double>* controller{nullptr};

  {
    auto pendulum_ptr = std::make_unique<PendulumPlant<double>>();
    pendulum = pendulum_ptr.get();
    pendulum->set_name("pendulum");

    // The choices of PidController constants here are fairly arbitrary,
    // but seem to effectively swing up the pendulum and hold it.
    controller =
        builder.AddSystem<systems::controllers::PidControlledSystem<double>>(
            std::move(pendulum_ptr), 10., 0., 1.);
    controller->set_name("controller");
  }
  DRAKE_DEMAND(pendulum != nullptr);
  DRAKE_DEMAND(controller != nullptr);

  // This is a fairly small number of time samples for this system,
  // and it winds up making the controller do a lot of the work when
  // getting to the target state.  I (sam.creasey) suspect that a
  // different interpolation strategy (not linear interpolation of a
  // non-linear system, basically) would reduce this effect.
  const int kNumTimeSamples = 21;
  const int kTrajectoryTimeLowerBound = 2;
  const int kTrajectoryTimeUpperBound = 6;

  const Eigen::Vector2d x0(0, 0);
  const Eigen::Vector2d xG(M_PI, 0);

  auto context = pendulum->CreateDefaultContext();

  systems::DircolTrajectoryOptimization dircol(
      pendulum, *context, kNumTimeSamples, kTrajectoryTimeLowerBound,
      kTrajectoryTimeUpperBound);
  drake::examples::pendulum::AddSwingUpTrajectoryParams(x0, xG, &dircol);

  const double timespan_init = 4;
  auto traj_init_x =
      PiecewisePolynomial<double>::FirstOrderHold({0, timespan_init}, {x0, xG});
  SolutionResult result = dircol.SolveTraj(
      timespan_init, PiecewisePolynomial<double>(), traj_init_x);
  if (result != SolutionResult::kSolutionFound) {
    std::cerr << "Result is an Error" << std::endl;
    return 1;
  }

  const PiecewisePolynomialTrajectory pp_traj =
      dircol.ReconstructInputTrajectory();
  const PiecewisePolynomialTrajectory pp_xtraj =
      dircol.ReconstructStateTrajectory();
  auto input_source = builder.AddSystem<systems::TrajectorySource>(pp_traj);
  input_source->set_name("input");
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  state_source->set_name("state");

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  builder.Connect(input_source->get_output_port(),
                  controller->get_input_port(0));
  builder.Connect(state_source->get_output_port(),
                  controller->get_input_port(1));
  builder.Connect(controller->get_output_port(0), publisher->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(pp_xtraj.get_end_time());

  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(
          *pendulum, simulator.get_mutable_context());
  auto state_vec = pendulum_context.get_continuous_state()->CopyToVector();
  if (!is_approx_equal_abstol(state_vec, xG, 1e-3)) {
    throw std::runtime_error("Did not reach trajectory target.");
  }
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pendulum::do_main();
}
