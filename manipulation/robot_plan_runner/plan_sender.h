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

  double get_all_plans_duration() const;

 private:
  void CalcPlan(const drake::systems::Context<double>&, PlanData*) const;
  void UpdatePlanIndex(const systems::Context<double>& context,
                       systems::State<double>* state) const;
  mutable std::vector<double> plan_start_times_{};
  mutable std::vector<PlanData> plan_data_list_{};
  mutable int num_plans_{0};
  systems::AbstractStateIndex abstract_state_index_{-1};
  int input_port_idx_q_{-1};
  const int num_positions_;
  const double transition_time_sec_;
  const double zoh_time_sec_;
  const double extra_time_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
