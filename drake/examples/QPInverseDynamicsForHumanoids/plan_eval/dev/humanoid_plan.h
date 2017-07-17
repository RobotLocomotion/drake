#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"
#include "drake/systems/controllers/zmp_planner.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template <typename T>
class HumanoidPlan : public GenericPlan<T> {
 public:
  HumanoidPlan() {}

 protected:
  GenericPlan<T>* CloneGenericPlanDerived() const;

  void InitializeGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups);

  void UpdateQpInputGenericPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups,
      QpInput* qp_input) const;

  void UpdateZmpPlan(const PiecewisePolynomial<T>& zmp_d,
                     const Vector4<T>& x_com0, const T height) {
    zmp_planner_.Plan(zmp_d, x_com0, height);
  }

  virtual HumanoidPlan<T>* CloneHumanoidPlanDerived() const = 0;

  virtual void InitializeHumanoidPlanDerived(
      const HumanoidStatus& robot_status,
      const param_parsers::ParamSet& paramset,
      const param_parsers::RigidBodyTreeAliasGroups<T>& alias_groups) = 0;

 private:
  systems::ZMPPlanner zmp_planner_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
