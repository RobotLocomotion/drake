#pragma once

#include <iostream>
#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_transcription.h"

using drake::multibody::MultibodyPlant;

namespace drake {
namespace examples {
namespace pendulum {

using trajectories::PiecewisePolynomial;

void AddSwingupConstraints(
    systems::trajectory_optimization::MultipleShooting* prog) {
  // TODO(russt): Add this constraint to PendulumPlant and get it automatically
  // through DirectCollocation.
  const double kTorqueLimit = 3.0;  // N*m.
  const solvers::VectorXDecisionVariable& u = prog->input();
  prog->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  prog->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  PendulumState<double> initial_state, final_state;
  initial_state.set_theta(0.0);
  initial_state.set_thetadot(0.0);
  final_state.set_theta(M_PI);
  final_state.set_thetadot(0.0);

  prog->AddLinearConstraint(prog->initial_state() == initial_state.get_value());
  prog->AddLinearConstraint(prog->final_state() == final_state.get_value());

  const double R = 10;  // Cost on input "effort".
  prog->AddRunningCost((R * u) * u);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state.get_value(), final_state.get_value()});
  prog->SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
}

bool ResultIsSuccess(solvers::MathematicalProgramResult result) {
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the swing-up trajectory"
              << std::endl;
    return false;
  } else {
    drake::log()->debug("Solved with solver {}", result.get_solver_id().name());
    return true;
  }
}

template <class Plant>
void SimulateTrajectory(
    const double realtime_rate, std::unique_ptr<Plant> pendulum,
    const geometry::SceneGraph<double>& scene_graph,
    const geometry::SourceId& source_id,
    const systems::trajectory_optimization::MultipleShooting& prog,
    const solvers::MathematicalProgramResult& result,
    systems::DiagramBuilder<double>* builder) {
  const PiecewisePolynomial<double> pp_traj =
      prog.ReconstructInputTrajectory(result);
  const PiecewisePolynomial<double> pp_xtraj =
      prog.ReconstructStateTrajectory(result);
  auto input_trajectory =
      builder->AddSystem<systems::TrajectorySource>(pp_traj);
  input_trajectory->set_name("input trajectory");
  auto state_trajectory =
      builder->AddSystem<systems::TrajectorySource>(pp_xtraj);
  state_trajectory->set_name("state trajectory");

  const int state_port_index =
      pendulum->get_continuous_state_output_port().get_index();
  const int control_input_port_index =
      pendulum->get_actuation_input_port().get_index();
  const int geometry_poses_port_index =
      pendulum->get_geometry_poses_output_port().get_index();
  const int geometry_query_port_index =
      pendulum->get_geometry_query_input_port().get_index();

  // The choices of PidController constants here are fairly arbitrary,
  // but seem to effectively swing up the pendulum and hold it.
  const double Kp = 10;
  const double Ki = 0;
  const double Kd = 1;
  auto pid_controlled_pendulum =
      builder->AddSystem<systems::controllers::PidControlledSystem<double>>(
          std::move(pendulum), Kp, Ki, Kd, state_port_index,
          control_input_port_index);
  pid_controlled_pendulum->set_name("PID Controlled Pendulum");

  builder->Connect(input_trajectory->get_output_port(),
                   pid_controlled_pendulum->get_control_input_port());
  builder->Connect(state_trajectory->get_output_port(),
                   pid_controlled_pendulum->get_state_input_port());
  builder->Connect(
      pid_controlled_pendulum->get_output_port(geometry_poses_port_index),
      scene_graph.get_source_pose_port(source_id));
  builder->Connect(
      scene_graph.get_query_output_port(),
      pid_controlled_pendulum->get_plant_input_port(geometry_query_port_index));

  geometry::ConnectDrakeVisualizer(builder, scene_graph);

  auto diagram = builder->Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(pp_xtraj.end_time());
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
