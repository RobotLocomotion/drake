#pragma once

#include <optional>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
/**
 * Impose the constraint p_EScm(q) - p_EC = 0, where p_EScm(q) is a function
 * that computes the center-of-mass (COM) position from robot generalized
 * position q, expressed in a frame E. p_EC ∈ ℝ³ is the variable representing
 * robot CoM (C) position expressed in frame E. The evaluated variables are
 * [q;r], where q is the generalized position vector of the entire plant.
 *
 * @ingroup solver_evaluators
 */
class ComPositionConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ComPositionConstraint)

  /**
   * Constructor, constrain f(q) = p_EC, where f(q) evaluates the CoM position
   * expressed in frame E using the generalized position q.
   * @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   * should be alive during the lifetime of this constraint.
   * @param model_instances We compute the model with these model instances in
   * `plant`. If model_instances=std::nullopt, then we compute the CoM position
   * of all model instances except the world.
   * @param expressed_frame The frame in which the CoM position is expressed.
   * @param plant_context The Context that has been allocated for this
   *   `plant`. We will update the context when evaluating the constraint.
   *   `plant_context` should be alive during the lifetime of this constraint.
   * @throws std::exception if `plant` or `plant_context` is nullptr.
   * @pydrake_mkdoc_identifier{ctor_double}
   */
  ComPositionConstraint(
      const MultibodyPlant<double>* const plant,
      std::optional<std::vector<ModelInstanceIndex>> model_instances,
      const Frame<double>& expressed_frame,
      systems::Context<double>* plant_context);

  /**
   * Overloaded constructor with MultibodyPlant<AutoDiffXd> and
   * Context<AutoDiffXd>.
   * It is preferrable to use the constructor with MBP<double> and
   * Context<double>. But if you only have MBP<AutoDiffXd> and
   * Context<AutoDiffXd>, then use this constructor.
   * @pydrake_mkdoc_identifier{ctor_autodiff}
   */
  ComPositionConstraint(
      const MultibodyPlant<AutoDiffXd>* const plant,
      std::optional<std::vector<ModelInstanceIndex>> model_instances,
      const Frame<AutoDiffXd>& expressed_frame,
      systems::Context<AutoDiffXd>* plant_context);

  ~ComPositionConstraint() override {}

  /**
   * Compose the variables for Eval function from generalized position q and the
   * CoM position p_EC.
   */
  template <typename T>
  void ComposeVariable(const Eigen::Ref<const VectorX<T>>& q,
                       const Eigen::Ref<const Vector3<T>>& p_EC,
                       VectorX<T>* variables) const {
    variables->resize(q.rows() + p_EC.rows());
    *variables << q, p_EC;
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "ComPositionConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  bool use_autodiff() const { return plant_autodiff_ != nullptr; }

  const MultibodyPlant<double>* const plant_double_;
  std::optional<std::vector<ModelInstanceIndex>> model_instances_;
  const FrameIndex expressed_frame_index_;
  systems::Context<double>* const context_double_;

  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* context_autodiff_;
};
}  // namespace multibody
}  // namespace drake
