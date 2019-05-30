#pragma once

#include <vector>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/manipulation/robot_plan_runner/plan_sender.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
/*
 * Connects RobotPlanRunner with PlanSender, iiwa lcm receiver and sender.
 * Runs Plans on the robot as well as mock_station_simulation.
 */
class PlanRunnerHardwareInterface {
 public:
  explicit PlanRunnerHardwareInterface(
      const std::vector<robot_plans::PlanData>&);
  /*
   * Saves the graphviz string which describes this system to a file.
   */
  void SaveGraphvizStringToFile(
      const std::string& file_name = "system_graphviz_string.txt");

  /*
   * Starts sending commands based on the list of PlanData with which this
   * class is constructed.
   */
  void Run(double realtime_rate = 1.0);
  lcmt_iiwa_status GetCurrentIiwaStatus();

 private:
  /*
   * Blocks until a iiwa_status lcm message is received.
   */
  void WaitForNewMessage(
      drake::lcm::DrakeLcmInterface* const lcm_ptr,
      systems::lcm::LcmSubscriberSystem* const lcm_sub) const;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<lcm::DrakeLcm> owned_lcm_;
  systems::lcm::LcmSubscriberSystem* iiwa_status_sub_;
  robot_plan_runner::PlanSender* plan_sender_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
