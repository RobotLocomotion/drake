#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans.h"

#include "drake/systems/framework/basic_vector.h"
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

  const int num_positions_{};
  std::unique_ptr<PlanBase> plan_;
  int input_port_idx_q_{-1};
  int input_port_idx_v_{-1};
  int input_port_idx_tau_ext_{-1};
  int input_port_idx_plan_data{-1};

  mutable Eigen::VectorXd q_cmd_;
  mutable Eigen::VectorXd tau_cmd_;
  mutable int plan_index_current_;

};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
