#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/systems/controllers/zmp_planner.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
class HumanoidManipulationPlan : public GenericPlan<T> {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HumanoidManipulationPlan)

 public:
  HumanoidManipulationPlan() : zmp_height_(1) {}

  double get_zmp_height() const { return zmp_height_; }

  void set_zmp_height(double z) { zmp_height_ = z; }

 private:
  GenericPlan<T>* CloneGenericPlanDerived() const override;

  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) override;

  // All the new desired trajectories will start with the current desired
  // states (dof, com, tracked body pose, etc). It is assumed that received
  // plan's keyframe time is specified in relative time to whenever the
  // plan is processed. Also the first timestamp needs to be bigger than
  // zero, and all subsequent timestamps need to be strictly increasing.
  // There must be at least 1 knots.
  // E.g. times = [0.1, 0.5, 0.6] is valid.
  // times = [0. 0.5] is not.
  void HandlePlanGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const systems::AbstractValue& plan) override;

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const override;

  void ModifyPlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) override {
  }

  systems::ZMPPlanner zmp_planner_;
  double zmp_height_;
  int64_t last_handle_plan_time_{-1};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
