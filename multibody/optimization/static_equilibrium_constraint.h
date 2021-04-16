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
 * Impose the static equilibrium constraint 0 = τ_g + Bu + ∑J_WBᵀ(q) * Fapp_B_W
 *
 * @ingroup solver_evaluators
 */
class StaticEquilibriumConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticEquilibriumConstraint)

  /**
   * Create a static equilibrium constraint
   * 0 = g(q) + Bu + ∑ᵢ JᵢᵀFᵢ_AB_W(λᵢ)
   * This constraint depends on the variables q, u and λ.
   * @param plant The plant on which the constraint is imposed.
   * @param context The context for the subsystem @p plant.
   * @param contact_wrench_evaluators_and_lambda For each contact pair, we
   * need to compute the contact wrench applied at the point of contact,
   * expressed in the world frame, namely Fᵢ_AB_W(λᵢ).
   * `contact_wrench_evaluators_and_lambda.first` is the evaluator for computing
   * this contact wrench from the variables λᵢ.
   * `contact_wrench_evaluators_and_lambda.second` are the decision variable λᵢ
   * used in computing the contact wrench. Notice the generalized position `q`
   * is not included in variables contact_wrench_evaluators_and_lambda.second.
   * @param q_vars The decision variables for q (the generalized position).
   * @param u_vars The decision variables for u (the input).
   * @return binding The binding between the static equilibrium constraint and
   * the variables q, u and λ.
   * @pre @p plant must have been connected to a SceneGraph properly. You could
   * refer to AddMultibodyPlantSceneGraph on how to connect a MultibodyPlant to
   * a SceneGraph.
   */
  static solvers::Binding<StaticEquilibriumConstraint> MakeBinding(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                                  VectorX<symbolic::Variable>>>&
          contact_wrench_evaluators_and_lambda,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& u_vars);

  ~StaticEquilibriumConstraint() override {}

  /**
   * Getter for contact_pair_to_wrench_evaluator, passed in the constructor.
   */
  const std::map<SortedPair<geometry::GeometryId>,
                 GeometryPairContactWrenchEvaluatorBinding>&
  contact_pair_to_wrench_evaluator() const {
    return contact_pair_to_wrench_evaluator_;
  }

 private:
  /**
   * The user cannot call this constructor, as it is inconvenient to do so.
   * The user must call MakeBinding() to construct a
   * StaticEquilibriumConstraint.
   */
  StaticEquilibriumConstraint(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::map<SortedPair<geometry::GeometryId>,
                     GeometryPairContactWrenchEvaluatorBinding>&
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
                 GeometryPairContactWrenchEvaluatorBinding>
      contact_pair_to_wrench_evaluator_;
  const MatrixX<AutoDiffXd> B_actuation_;
};

}  // namespace multibody
}  // namespace drake
