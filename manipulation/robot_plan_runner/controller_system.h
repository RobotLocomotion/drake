#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

/*
 * A Drake System wrapper for robot_plans, which calls Plan::Step() when the
 * output port of this system is evaluated.
 *
 * control_period is ignored by JoinstSpacePlan.
 * control_period is used to in TaskSpacePlan to calculate joint angle updates
 * from desired joint velocities.
 */
class RobotController : public systems::LeafSystem<double> {
 public:
  explicit RobotController(
      robot_plans::PlanType plan_type, double control_period=0.);
  robot_plans::PlanType get_plan_type();
 private:
  void CalcCommands(const systems::Context<double>&,
                       systems::BasicVector<double>*) const;
  std::unique_ptr<robot_plans::PlanBase> plan_;
  const int num_positions_{};
  const double control_period_{};
  int input_port_idx_q_{-1};
  int input_port_idx_v_{-1};
  int input_port_idx_tau_ext_{-1};
  int input_port_idx_plan_data_{-1};

  mutable Eigen::VectorXd q_cmd_;
  mutable Eigen::VectorXd tau_cmd_;
  mutable double t_start_current_{0};
  mutable long plan_signature_current_{0};
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
