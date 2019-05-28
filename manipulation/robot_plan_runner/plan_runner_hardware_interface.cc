#include <fstream>

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using std::cout;
using std::endl;
using robot_plans::PlanData;

PlanRunnerHardwareInterface::PlanRunnerHardwareInterface(
    const std::vector<PlanData>& plan_list)
    : owned_lcm_(new lcm::DrakeLcm()) {
  // create diagram system.
  systems::DiagramBuilder<double> builder;

  auto lcm =
      builder.AddSystem<systems::lcm::LcmInterfaceSystem>(owned_lcm_.get());

  // Receive iiwa status.
  iiwa_status_sub_ = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));
  auto iiwa_status_receiver =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaStatusReceiver>();
  builder.Connect(iiwa_status_sub_->get_output_port(),
                  iiwa_status_receiver->get_input_port());

  // Publish iiwa command.
  auto iiwa_command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm, 0.005));
  auto iiwa_command_sender =
      builder.AddSystem<manipulation::kuka_iiwa::IiwaCommandSender>();
  builder.Connect(iiwa_command_sender->get_output_port(),
                  iiwa_command_pub->get_input_port());

  // Add PlanSender and PlanRunner.
  plan_sender_ = builder.AddSystem<PlanSender>(plan_list);
  auto plan_runner = builder.AddSystem<RobotPlanRunner>(0.);
  builder.Connect(plan_sender_->GetOutputPort("plan_data"),
                  plan_runner->GetInputPort("plan_data"));

  builder.Connect(iiwa_status_receiver->get_position_measured_output_port(),
                  plan_sender_->GetInputPort("q"));
  builder.Connect(iiwa_status_receiver->get_position_measured_output_port(),
                  plan_runner->GetInputPort("iiwa_position_measured"));
  builder.Connect(iiwa_status_receiver->get_velocity_estimated_output_port(),
                  plan_runner->GetInputPort("iiwa_velocity_estimated"));
  builder.Connect(iiwa_status_receiver->get_torque_external_output_port(),
                  plan_runner->GetInputPort("iiwa_torque_external"));

  builder.Connect(plan_runner->GetOutputPort("iiwa_position_command"),
                  iiwa_command_sender->get_position_input_port());
  builder.Connect(plan_runner->GetOutputPort("iiwa_torque_command"),
                  iiwa_command_sender->get_torque_input_port());

  diagram_ = builder.Build();
};

void PlanRunnerHardwareInterface::SaveGraphvizStringToFile(
    const std::string& file_name) {
  if (diagram_) {
    std::ofstream out(file_name);
    out << diagram_->GetGraphvizString();
    out.close();
  }
}

/*
 * Get current iiwa status by creating a diagram with only a lcm subscriber
 * and simulating it for 1e-6 seconds.
 */
lcmt_iiwa_status PlanRunnerHardwareInterface::GetCurrentIiwaStatus() {
  // create diagram system.
  systems::DiagramBuilder<double> builder;
  auto lcm =
      builder.AddSystem<systems::lcm::LcmInterfaceSystem>(new lcm::DrakeLcm());

  // Receive iiwa status.
  auto iiwa_status_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  this->WaitForNewMessage(lcm, iiwa_status_sub);
  simulator.AdvanceTo(1e-6);

  const auto& iiwa_status_sub_context = diagram->GetMutableSubsystemContext(
      *iiwa_status_sub, &simulator.get_mutable_context());

  return iiwa_status_sub->get_output_port().Eval<lcmt_iiwa_status>(
      iiwa_status_sub_context);
}

void PlanRunnerHardwareInterface::WaitForNewMessage(
    drake::lcm::DrakeLcmInterface* const lcm_ptr,
    systems::lcm::LcmSubscriberSystem* const lcm_sub_ptr) const {
  auto wait_for_new_message = [lcm_ptr](const auto& lcm_sub) {
    std::cout << "Waiting for " << lcm_sub.get_channel_name() << " message..."
              << std::flush;
    const int orig_count = lcm_sub.GetInternalMessageCount();
    LcmHandleSubscriptionsUntil(
        lcm_ptr,
        [&]() { return lcm_sub.GetInternalMessageCount() > orig_count; },
        10 /* timeout_millis */);
    std::cout << "Received!" << std::endl;
  };

  wait_for_new_message(*lcm_sub_ptr);
};

void PlanRunnerHardwareInterface::Run(double realtime_rate) {
  systems::Simulator<double> simulator(*diagram_);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(realtime_rate);

  // Update the abstract state of iiwa status lcm subscriber system, so that
  // actual robot state can be obtained when its output ports are evaluated at
  // initialization.
  auto& iiwa_status_sub_context = diagram_->GetMutableSubsystemContext(
      *iiwa_status_sub_, &simulator.get_mutable_context());
  auto& state = iiwa_status_sub_context
      .get_mutable_abstract_state<lcmt_iiwa_status>(0);
  state = this->GetCurrentIiwaStatus();

  double t_total = plan_sender_->get_all_plans_duration();
  cout << "All plans duration " << t_total << endl;
  simulator.AdvanceTo(t_total);
}

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
