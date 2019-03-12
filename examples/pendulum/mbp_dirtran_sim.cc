#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

#include "drake/common/unused.h"

using drake::solvers::SolutionResult;
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

int DoMain() {
  double dt = 0.01;
  const char* const urdf_path = "drake/examples/pendulum/Pendulum.urdf";
  auto pendulum = std::make_unique<MultibodyPlant<double>>(dt);
  pendulum->AddForceElement<UniformGravityFieldElement>();
  Parser parser(pendulum.get());
  parser.AddModelFromFile(FindResourceOrThrow(urdf_path));
  pendulum->WeldFrames(pendulum->world_frame(),
                       pendulum->GetFrameByName("base_part2"));

  pendulum->Finalize();
  pendulum->set_name("pendulum");

  auto context = pendulum->CreateDefaultContext();
  const int actuation_port_index =
      pendulum->get_actuation_input_port().get_index();

  const int kNumTimeSamples = 500;
  systems::trajectory_optimization::DirectTranscription dirtran(
      pendulum.get(), *context, kNumTimeSamples, actuation_port_index);

  const solvers::VectorXDecisionVariable& u = dirtran.input();
  const double kTorqueLimit = 3.0;  // N*m.
  dirtran.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dirtran.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  PendulumState<double> initial_state, final_state;
  initial_state.set_theta(0.0);
  initial_state.set_thetadot(0.0);
  final_state.set_theta(M_PI);
  final_state.set_thetadot(0.0);

  dirtran.AddLinearConstraint(dirtran.initial_state() ==
                             initial_state.get_value());
  dirtran.AddLinearConstraint(dirtran.final_state() == final_state.get_value());

  const double R = 10;  // Cost on input "effort".
  dirtran.AddRunningCost((R * u) * u);

  const double timespan_init = 5;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state.get_value(), final_state.get_value()});
  dirtran.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  const auto result = solvers::Solve(dirtran);
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the swing-up trajectory"
              << " using solver "<< result.get_solver_id().name() << std::endl;
    std::cerr << "Solution result: " << result.get_solution_result();
    return 1;
  } else {
    std::cerr << "Solved with solver " << result.get_solver_id().name()
              << std::endl;
  }

  // Now, simulate the result, with the continuous time plant.
  systems::DiagramBuilder<double> builder;

  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  auto spendulum = std::make_unique<MultibodyPlant<double>>();
  spendulum->AddForceElement<UniformGravityFieldElement>();
  Parser sparser(spendulum.get(), scene_graph);
  sparser.AddModelFromFile(FindResourceOrThrow(urdf_path));
  spendulum->WeldFrames(spendulum->world_frame(),
                        spendulum->GetFrameByName("base_part2"));

  const geometry::SourceId source_id = spendulum->get_source_id().value();

  spendulum->Finalize(scene_graph);
  spendulum->set_name("spendulum");

  const PiecewisePolynomial<double> pp_traj =
      dirtran.ReconstructInputTrajectory(result);
  const PiecewisePolynomial<double> pp_xtraj =
      dirtran.ReconstructStateTrajectory(result);
  auto input_trajectory = builder.AddSystem<systems::TrajectorySource>(pp_traj);
  input_trajectory->set_name("input trajectory");
  auto state_trajectory =
      builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  state_trajectory->set_name("state trajectory");

  const int state_port_index =
      spendulum->get_continuous_state_output_port().get_index();
  const int control_input_port =
      spendulum->get_actuation_input_port().get_index();
  const int geometry_port_index =
      spendulum->get_geometry_poses_output_port().get_index();

  // The choices of PidController constants here are fairly arbitrary,
  // but seem to effectively swing up the pendulum and hold it.
  const double Kp = 10;
  const double Ki = 0;
  const double Kd = 1;
  auto pid_controlled_pendulum =
      builder.AddSystem<systems::controllers::PidControlledSystem<double>>(
          std::move(spendulum), Kp, Ki, Kd, state_port_index,
          control_input_port);
  pid_controlled_pendulum->set_name("PID Controlled Pendulum");

  builder.Connect(input_trajectory->get_output_port(),
                  pid_controlled_pendulum->get_control_input_port());
  builder.Connect(state_trajectory->get_output_port(),
                  pid_controlled_pendulum->get_state_input_port());
  builder.Connect(pid_controlled_pendulum->get_output_port(geometry_port_index),
                  scene_graph->get_source_pose_port(source_id));

  // TODO(rcory) Make proper geom query port accessor for PidControlledSystem
  // with MBP.
  builder.Connect(scene_graph->get_query_output_port(),
      pid_controlled_pendulum->get_input_port(2));

  geometry::ConnectDrakeVisualizer(&builder, *scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(pp_xtraj.end_time());

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
