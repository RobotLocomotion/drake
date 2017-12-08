#include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"

#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

SchunkWsgController::SchunkWsgController() {
  systems::DiagramBuilder<double> builder;

  auto wsg_trajectory_generator =
      builder.AddSystem<SchunkWsgTrajectoryGenerator>(
          kSchunkWsgNumPositions + kSchunkWsgNumVelocities,
          kSchunkWsgPositionIndex);
  command_input_port_ = builder.ExportInput(
      wsg_trajectory_generator->get_command_input_port());

  auto state_pass_through =
      builder.AddSystem<systems::PassThrough<double>>(
          kSchunkWsgNumPositions + kSchunkWsgNumVelocities);

  state_input_port_ =
      builder.ExportInput(state_pass_through->get_input_port());
  builder.Connect(state_pass_through->get_output_port(),
                  wsg_trajectory_generator->get_state_input_port());

  const int kWsgActDim = kSchunkWsgNumActuators;
  // The p gain here is somewhat arbitrary.  The goal is to make sure
  // that the maximum force is generated except when very close to the
  // target.
  const Eigen::VectorXd wsg_kp = Eigen::VectorXd::Constant(kWsgActDim, 2000.0);
  const Eigen::VectorXd wsg_ki = Eigen::VectorXd::Constant(kWsgActDim, 0.0);
  const Eigen::VectorXd wsg_kd = Eigen::VectorXd::Constant(kWsgActDim, 5.0);

  auto wsg_controller =
      builder.AddSystem<systems::controllers::PidController<double>>(
          GetSchunkWsgFeedbackSelector<double>(),
          wsg_kp, wsg_ki, wsg_kd);

  builder.Connect(state_pass_through->get_output_port(),
                  wsg_controller->get_input_port_estimated_state());
  builder.Connect(wsg_trajectory_generator->get_target_output_port(),
                  wsg_controller->get_input_port_desired_state());

  // Create a gain block to negate the max force (to produce a minimum
  // force).
  auto gain = builder.AddSystem<systems::Gain<double>>(-1.0, 1);
  builder.Connect(wsg_trajectory_generator->get_max_force_output_port(),
                  gain->get_input_port());

  auto saturation = builder.AddSystem<systems::Saturation<double>>(1);
  builder.Connect(wsg_controller->get_output_port_control(),
                  saturation->get_input_port());
  builder.Connect(wsg_trajectory_generator->get_max_force_output_port(),
                  saturation->get_max_value_port());
  builder.Connect(gain->get_output_port(),
                  saturation->get_min_value_port());
  builder.ExportOutput(saturation->get_output_port());
  builder.BuildInto(this);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
