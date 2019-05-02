#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/manipulation/robot_plan_runner/controller_systems.h"
#include "drake/manipulation/robot_plan_runner/robot_plans.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

RobotPlanRunner::RobotPlanRunner(double control_period_sec) {
  this->set_name("RobotPlanRunner");
  systems::DiagramBuilder<double> builder;
  int num_positions = 7;

  // input pass-throughs.
  auto passthrough_plan_data =
      builder.template AddSystem<systems::PassThrough>(Value<PlanData>{});
  builder.ExportInput(passthrough_plan_data->get_input_port(), "plan_data");
  passthrough_plan_data->set_name("PassThrough_plan_data");

  auto passthrough_q_measured =
      builder.template AddSystem<systems::PassThrough>(num_positions);
  builder.ExportInput(passthrough_q_measured->get_input_port(),
                      "iiwa_position_measured");
  passthrough_q_measured->set_name("PassThrough_q");

  auto passthrough_v_estimated =
      builder.template AddSystem<systems::PassThrough>(num_positions);
  builder.ExportInput(passthrough_v_estimated->get_input_port(),
                      "iiwa_velocity_estimated");
  passthrough_v_estimated->set_name("PassThrough_v");

  auto passthough_tau_external =
      builder.template AddSystem<systems::PassThrough>(num_positions);
  builder.ExportInput(passthough_tau_external->get_input_port(),
                      "iiwa_torque_external");
  passthough_tau_external->set_name("PassThrough_tau_external");

  // controller systems.
  auto joint_space_controller =
      builder.template AddSystem<RobotController>(PlanType::kJointSpacePlan);

  builder.Connect(passthrough_plan_data->get_output_port(),
                  joint_space_controller->GetInputPort("plan_data"));
  builder.Connect(passthrough_q_measured->get_output_port(),
                  joint_space_controller->GetInputPort("q"));
  builder.Connect(passthrough_v_estimated->get_output_port(),
                  joint_space_controller->GetInputPort("v"));
  builder.Connect(passthough_tau_external->get_output_port(),
                  joint_space_controller->GetInputPort("tau_external"));

  // output ZOH and demux.
  auto output_zoh = builder.template AddSystem<systems::ZeroOrderHold>(
      control_period_sec, num_positions * 2);
  auto demux = builder.template AddSystem<systems::Demultiplexer>(
      2 * num_positions, num_positions);
  builder.Connect(joint_space_controller->GetOutputPort("q_tau_cmd"),
                  output_zoh->get_input_port());
  builder.Connect(output_zoh->get_output_port(), demux->get_input_port(0));
  builder.ExportOutput(demux->get_output_port(0), "iiwa_position_command");
  builder.ExportOutput(demux->get_output_port(1), "iiwa_torque_command");

  builder.BuildInto(this);
}

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
