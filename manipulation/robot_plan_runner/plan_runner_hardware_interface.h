#pragma once

#include <vector>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/robot_plan_runner/robot_plans.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

class PlanRunnerHardwareInterface {
 public:
  explicit PlanRunnerHardwareInterface(const std::vector<PlanData>&);
  void SaveGraphvizStringToFile(
      const std::string& file_name = "system_graphviz_string.txt");
  void Run();
  lcmt_iiwa_status GetCurrentIiwaStatus();

 private:
  void WaitForNewMessage(
      drake::lcm::DrakeLcmInterface* const lcm_ptr,
      systems::lcm::LcmSubscriberSystem* const lcm_sub) const;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<lcm::DrakeLcm> owned_lcm_;
  systems::lcm::LcmSubscriberSystem* iiwa_status_sub_;
  Eigen::VectorXd q_current_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
