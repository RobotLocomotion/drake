#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/dev/humanoid_plan.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
class HumanoidManipulationPlan : public HumanoidPlan<T> {
 public:
  HumanoidManipulationPlan() {}

 private:
  HumanoidPlan<T>* CloneHumanoidPlanDerived() const;

  void InitializeHumanoidPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void HandlePlanMessageGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      const void* message_bytes, int message_length);

  void ModifyPlanGenericPlanDerived(
      const HumanoidStatus& robot_stauts,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  const manipulation::RobotStateLcmMessageTranslator translator_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
