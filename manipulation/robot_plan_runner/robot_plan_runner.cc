#include <vector>

#include "drake/manipulation/robot_plan_runner/controller_system.h"
#include "drake/manipulation/robot_plan_runner/plan_switcher.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/port_switch.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using robot_plans::PlanData;
using robot_plans::PlanType;

RobotPlanRunner::RobotPlanRunner(bool is_discrete, double control_period_sec) {
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

  // Add controller systems in the order they appear in PlanType.
  std::vector<RobotController*> controllers;
  for(int i = 1; i < static_cast<int>(PlanType::kLastElement); i++) {
    auto controller = builder.template AddSystem<RobotController>(
        static_cast<PlanType>(i), control_period_sec);
    controllers.push_back(controller);
  }

  // PortSwitch system.
  auto port_switch = builder.template AddSystem<systems::PortSwitch<double>>(
      controllers[0]->GetOutputPort("q_tau_cmd").size());
  port_switch->set_name("port_switch");

  for (const auto controller : controllers) {
    // connection to passthroughs
    builder.Connect(passthrough_plan_data->get_output_port(),
                    controller->GetInputPort("plan_data"));
    builder.Connect(passthrough_q_measured->get_output_port(),
                    controller->GetInputPort("q"));
    builder.Connect(passthrough_v_estimated->get_output_port(),
                    controller->GetInputPort("v"));
    builder.Connect(passthough_tau_external->get_output_port(),
                    controller->GetInputPort("tau_external"));

    // connection to port switch
    std::string port_name =
        "q_tau_cmd_" +
        std::to_string(static_cast<int>(controller->get_plan_type()));
    port_switch->DeclareInputPort(port_name);
    builder.Connect(controller->GetOutputPort("q_tau_cmd"),
                    port_switch->GetInputPort(port_name));
  }

  // plan switcher
  auto plan_switcher = builder.template AddSystem<PlanSwitcher>();
  builder.Connect(passthrough_plan_data->get_output_port(),
                  plan_switcher->GetInputPort("plan_data"));
  builder.Connect(plan_switcher->GetOutputPort("port_switch_index"),
                  port_switch->get_port_selector_input_port());

  // output ZOH and demux.
  auto demux = builder.template AddSystem<systems::Demultiplexer>(
      2 * num_positions, num_positions);
  demux->set_name("demux");
  builder.ExportOutput(demux->get_output_port(0), "iiwa_position_command");
  builder.ExportOutput(demux->get_output_port(1), "iiwa_torque_command");

  if (is_discrete) {
    auto output_zoh =
        builder.template AddSystem<systems::ZeroOrderHold<double>>(
            control_period_sec, num_positions * 2);
    builder.Connect(port_switch->get_output_port(),
                    output_zoh->get_input_port());
    builder.Connect(output_zoh->get_output_port(), demux->get_input_port(0));
  } else {
    builder.Connect(port_switch->get_output_port(), demux->get_input_port(0));
  }

  builder.BuildInto(this);
}

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
