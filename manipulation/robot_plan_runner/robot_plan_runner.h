# pragma once

#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

class RobotPlanRunner : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotPlanRunner);
  explicit RobotPlanRunner(double control_period_sec = 0.005);
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake