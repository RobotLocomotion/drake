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

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length) override;

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const override;

  void ModifyPlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) override {}

  systems::ZMPPlanner zmp_planner_;
  double zmp_height_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
