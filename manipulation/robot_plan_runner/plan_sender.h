#pragma once

#include <vector>

#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

/*
 * Constructed with a list of PlanData, this system puts PlanData one by
 * one on its output port for the duration of each PlanData.
 */
class PlanSender : public systems::LeafSystem<double> {
 public:
  explicit PlanSender(const std::vector<robot_plans::PlanData>&);

  /*
   * Initialization event callback function.
   * It Inserts the following plans to the beginning of the given list of
   * PlanData.
   * 1) a ZeroOrderHold which holds the robot's current position for
   *    zoh_time_sec_.
   * 2) A JointSpacePlan that interpolates from the current joint angles to
   *    the joint angles at the beginning of the first plan in the given
   *    PlanData list, if the first plan is also JointSpacePlan. The duration
   *    of this plan is transition_time_sec_.
   *    This interpolation is not needed for TaskSpacePlans because they are
   *    defined relative to current end effector position.
   */
  systems::EventStatus Initialize(const systems::Context<double>& context,
                                  systems::State<double>* state) const;

  void DoCalcNextUpdateTime(const systems::Context<double>&,
                            systems::CompositeEventCollection<double>*,
                            double* time) const override;
  /*
   * Returns the duration of all plans in the
   */
  double get_all_plans_duration() const;

 private:
  void CalcPlan(
      const drake::systems::Context<double>&, robot_plans::PlanData*) const;
  void UpdatePlanIndex(const systems::Context<double>& context,
                       systems::State<double>* state) const;
  mutable std::vector<double> plan_start_times_{};
  mutable std::vector<robot_plans::PlanData> plan_data_list_{};
  mutable int num_plans_{0};
  systems::AbstractStateIndex abstract_state_index_;
  int input_port_idx_q_{-1};
  const int num_positions_;
  const double transition_time_sec_;
  const double zoh_time_sec_;
  const double extra_time_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
