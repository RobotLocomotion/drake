#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

/*
 * This system takes in PlanData and converts the input's PlanType to an
 * InputPortIndex, which is then passed to a PortSwitch that decides which
 * controller system to use.
 */
class PlanSwitcher : public systems::LeafSystem<double> {
 public:
  PlanSwitcher();

 private:
  void CalcSwitchIndex(const systems::Context<double>&,
      systems::InputPortIndex *) const;
  //  const int num_positions_;
  int input_port_idx_plan_data_;
  //  mutable long plan_signature_current_{0};
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
