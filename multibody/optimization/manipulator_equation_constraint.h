#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {

/**
 * A Constraint to impose the manipulator equation:
 * 0 = (Buₙ₊₁ + ∑ᵢ (Jᵢ_WBᵀ(qₙ₊₁)ᵀ * Fᵢ_AB_W(λᵢ,ₙ₊₁))
 *     + tau_g(qₙ₊₁) - C(qₙ₊₁, Vₙ₊₁)) * dt - M(qₙ₊₁) * (Vₙ₊₁ - Vₙ)
 */
class ManipulatorEquationConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulatorEquationConstraint)

  /**
   * This constraint depends on the decision variable vector:
   * {vₙ, qₙ₊₁, vₙ₊₁, uₙ₊₁, λₙ₊₁, dt}.
   * @param plant The plant on which the constraint is imposed.
   * @param context The context for the subsystem @p plant. This context stores
   * the next state {qₙ₊₁, vₙ₊₁}.
   * @param contact_wrench_evaluators_and_lambda For each contact pair, we
   * need to compute the contact wrench applied at the point of contact,
   * expressed in the world frame, namely Fᵢ_AB_W(λᵢ,ₙ₊₁) at time n+1.
   * `contact_wrench_evaluators_and_lambda.first` is the evaluator for computing
   * this contact wrench from the variables λᵢ[.].
   * `contact_wrench_evaluators_and_lambda.second` are the decision variable
   * λᵢ[n+1] used in computing the contact wrench at time step n+1. Notice the
   * generalized position `q` is not included in variables
   * contact_wrench_evaluators_and_lambda.second.
   * @param v_vars The decision variables for vₙ.
   * @param q_next_vars The decision variables for qₙ₊₁.
   * @param v_next_vars The decision variables for vₙ₊₁.
   * @param u_next_vars The decision variables for uₙ₊₁.
   * @param dt_var The decision variable for dt.
   * @return binding The binding between the manipulator equation constraint
   * and the variables vₙ, qₙ₊₁, vₙ₊₁, uₙ₊₁, λₙ₊₁, and dt.
   * @pre @p plant must have been connected to a SceneGraph properly. Refer to
   * AddMultibodyPlantSceneGraph for documentation on connecting a
   * MultibodyPlant to a SceneGraph.
   */
  static solvers::Binding<ManipulatorEquationConstraint> MakeBinding(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                                  VectorX<symbolic::Variable>>>&
          contact_wrench_evaluators_and_lambda,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& v_vars,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& q_next_vars,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& v_next_vars,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& u_next_vars,
      const symbolic::Variable& dt_var);

  ~ManipulatorEquationConstraint() override {}

  /**
   * Getter for contact_pair_to_wrench_evaluator, passed in the constructor.
   */
  const std::map<SortedPair<geometry::GeometryId>,
                 internal::GeometryPairContactWrenchEvaluatorBinding>&
  contact_pair_to_wrench_evaluator() const {
    return contact_pair_to_wrench_evaluator_;
  }

 private:
  /**
   * Users do not call this constructor explicitly, and will instead call
   * MakeBinding() to construct a ManipulatorEquationConstraint.
   */
  ManipulatorEquationConstraint(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::map<SortedPair<geometry::GeometryId>,
                     internal::GeometryPairContactWrenchEvaluatorBinding>&
          contact_pair_to_wrench_evaluator);

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final;
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final;
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final;

  const MultibodyPlant<AutoDiffXd>* const plant_;
  systems::Context<AutoDiffXd>* const context_;
  const std::map<SortedPair<geometry::GeometryId>,
                 internal::GeometryPairContactWrenchEvaluatorBinding>
      contact_pair_to_wrench_evaluator_;
  const MatrixX<AutoDiffXd> B_actuation_;
};

}  // namespace multibody
}  // namespace drake
