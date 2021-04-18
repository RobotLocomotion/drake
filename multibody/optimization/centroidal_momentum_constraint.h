#pragma once

#include <optional>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Impose the constraint CentroidalMomentum(q, v) - h_WC = 0.
 * The decision variables are [q;v;h]
 * h_WC is the 6D spatial momentum (linear and angular momentum about the
 * center of mass C) expressed in the world frame (W).
 *
 * @ingroup solver_evaluators
 */
class CentroidalMomentumConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CentroidalMomentumConstraint)

  /**
   * Construct centroidal momentum constraint
   * CentroidalMomentum(q, v) - h_WC = 0
   * where CentroidalMomentum computes the spatial momentum of the robot
   * about its center-of-mass, expressed in the world frame.
   * The decision variables are [q;v;h_WC].
   * @param plant The plant for which the constraint is imposed.
   * @param model_instances We compute the model with these model instances in
   * `plant`. If model_instances=std::nullopt, then we compute the momentum
   * of all model instances except the world.
   */
  CentroidalMomentumConstraint(
      const MultibodyPlant<AutoDiffXd>* plant,
      std::optional<std::vector<ModelInstanceIndex>> model_instances,
      systems::Context<AutoDiffXd>* plant_context);

  ~CentroidalMomentumConstraint() override {}

  template <typename T>
  void ComposeVariable(const Eigen::Ref<const VectorX<T>>& q,
                       const Eigen::Ref<const VectorX<T>>& v,
                       const Eigen::Ref<const Vector6<T>>& h,
                       VectorX<T>* vars) const {
    vars->resize(q.rows() + v.rows() + 6);
    *vars << q, v, h;
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "CentroidalMomentumConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  bool use_autodiff() const { return plant_autodiff_; }

  std::optional<std::vector<ModelInstanceIndex>> model_instances_;
  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* const context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
