#pragma once

#include <vector>

#include "drake/manipulation/robot_plan_runner/robot_plans.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

class PlanSender : public systems::LeafSystem<double> {
 public:
  explicit PlanSender(const std::vector<PlanData>&);

  systems::EventStatus Initialize(const systems::Context<double>& context,
                                  systems::State<double>* state) const;

  void DoCalcNextUpdateTime(const systems::Context<double>&,
                            systems::CompositeEventCollection<double>*,
                            double* time) const override;

 private:
  void CalcPlan(const drake::systems::Context<double>& context,
                PlanData* output_plan_data) const;
  mutable std::vector<double> plan_start_times_{};
  mutable std::vector<PlanData> plan_data_list_{};
  mutable int current_plan_idx_{-1};
  mutable int num_plans_{0};
  systems::AbstractStateIndex abstract_state_index_{-1};
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
