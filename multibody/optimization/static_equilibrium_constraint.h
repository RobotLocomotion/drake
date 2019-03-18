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

struct GeometryPairContactWrenchEvaluatorBinding {
  GeometryPairContactWrenchEvaluatorBinding(
      std::vector<int> lambda_indices_in_all_lambda_in,
      std::shared_ptr<ContactWrenchEvaluator> contact_wrench_evaluator_in)
      : lambda_indices_in_all_lambda{std::move(
            lambda_indices_in_all_lambda_in)},
        contact_wrench_evaluator{std::move(contact_wrench_evaluator_in)} {
    DRAKE_DEMAND(static_cast<int>(lambda_indices_in_all_lambda.size()) ==
                 contact_wrench_evaluator->num_lambda());
  }
  std::vector<int> lambda_indices_in_all_lambda;
  std::shared_ptr<ContactWrenchEvaluator> contact_wrench_evaluator;
};

/**
 * Impose the constraint 0 = τ_g + Bu + ∑J_WBᵀ(q) Fapp_B_W
 */
class StaticEquilibriumConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticEquilibriumConstraint)

  StaticEquilibriumConstraint(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::map<std::pair<geometry::GeometryId, geometry::GeometryId>,
                     GeometryPairContactWrenchEvaluatorBinding>&
          contact_pair_to_wrench_evaluator);

  ~StaticEquilibriumConstraint() override {}

  /**
   * Getter for contact_pair_to_wrench_evaluator, passed in the constructor.
   */
  const std::map<std::pair<geometry::GeometryId, geometry::GeometryId>,
                 GeometryPairContactWrenchEvaluatorBinding>&
  contact_pair_to_wrench_evaluator() const {
    return contact_pair_to_wrench_evaluator_;
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;
  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  const MultibodyPlant<AutoDiffXd>* const plant_;
  systems::Context<AutoDiffXd>* const context_;
  const std::map<std::pair<geometry::GeometryId, geometry::GeometryId>,
                 GeometryPairContactWrenchEvaluatorBinding>
      contact_pair_to_wrench_evaluator_;
  const MatrixX<AutoDiffXd> B_actuation_;
};

/**
 * Create a static equilibrium constraint
 * 0 = g(q) + Bu + ∑ᵢ JᵢᵀFᵢ_AB_W(λᵢ)
 * This constraint depends on the variable q, u and λ.
 * @param plant The plant on which the constraint is imposed.
 * @param context The context for the subsystem @p plant.
 * @param contact_wrench_evaluators_and_lambda For each pair of contact, we need
 * to compute the contact wrench applied at the point of contact, expressed in
 * the world frame, namely Fᵢ_AB_W(λᵢ). @p
 * contact_wrench_evaluators_and_lambda.first is the evaluator for computing
 * this contact wrench from the variables λᵢ. @p
 * contact_wrench_evaluators_and_lambda.second are the decision variable λᵢ used
 * in computing the contact wrench.
 * @param q_vars The decision variables for q (the generalized position).
 * @param u_vars The decision variables for u (the input).
 * @return binding The binding between the static equilibrium constraint and the
 * variables q, u and λ.
 * @pre @p plant must have been connected to a SceneGraph properly. You could
 * refer to AddMultibodyPlantSceneGraph on how to connect a MultibodyPlant to a
 * SceneGraph.
 */
solvers::Binding<StaticEquilibriumConstraint> CreateStaticEquilibriumConstraint(
    const MultibodyPlant<AutoDiffXd>* plant,
    systems::Context<AutoDiffXd>* context,
    const std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                                VectorX<symbolic::Variable>>>&
        contact_wrench_evaluators_and_lambda,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& u_vars);
}  // namespace multibody
}  // namespace drake
