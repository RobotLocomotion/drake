#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

class RobotController : public systems::LeafSystem<double> {
 public:
  RobotController(PlanType plan_type);

 private:
  void CalcCommands(const systems::Context<double>&,
                       systems::BasicVector<double>*) const;
  std::unique_ptr<PlanBase> plan_;
  const int num_positions_{};
  int input_port_idx_q_{-1};
  int input_port_idx_v_{-1};
  int input_port_idx_tau_ext_{-1};
  int input_port_idx_plan_data_{-1};

  mutable Eigen::VectorXd q_cmd_;
  mutable Eigen::VectorXd tau_cmd_;
  mutable double t_start_current_{0};
  mutable long plan_signature_current_{0};
};

class PlanSwitcher : public systems::LeafSystem<double> {
 public:
  PlanSwitcher();
 private:
  void CalcJointSpacePlan(const systems::Context<double>&,
                          PlanData *) const;
//  const int num_positions_;
  int input_port_idx_plan_data_;
//  mutable long plan_signature_current_{0};
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
