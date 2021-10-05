#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/multibody/optimization/contact_wrench.h"
#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/multibody/optimization/static_friction_cone_complementarity_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace multibody {
// TODO(hongkai.dai): add the bounds on the input u, and other position
// constraint (such as unit length constraint on quaternion).
/**
 * Finds the static equilibrium pose of a multibody system through optimization.
 * The constraints are
 * 1. 0 = g(q) + Bu + ∑ᵢ JᵢᵀFᵢ_AB_W(λᵢ) (generalized force equals to 0).
 * 2. Fᵢ_AB_W(λᵢ) is within the admissible contact wrench (for example, contact
 *    force is in the friction cone).
 * 3. sdf(q) >= 0 (signed distance function is no smaller than 0, hence no
 *    penetration).
 * 4. complementarity condition between the contact force and the signed
 *    distance.
 * 5. q within the joint limit.
 *
 * @ingroup planning
 */
class StaticEquilibriumProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticEquilibriumProblem)

  /**
   * @param plant The plant for which the static equilibrium posture is
   * computed. @p plant should remain alive as long as this
   * StaticEquilibriumProblem exists.
   * @param context The context for `plant`. @p context should remain alive as
   * long as this StaticEquilibriumProblem exists.
   * @param ignored_collision_pairs The contact between the pair of geometry in
   * `ignored_collision_pairs` will be ignored. We will not impose
   * non-penetration constraint between these pairs, and no contact wrench will
   * be applied between these pairs.
   */
  StaticEquilibriumProblem(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>&
          ignored_collision_pairs);

  ~StaticEquilibriumProblem() {}

  solvers::MathematicalProgram* get_mutable_prog() const { return prog_; }

  /** Getter for the immutable optimization program. */
  const solvers::MathematicalProgram& prog() const { return *prog_; }

  /** Getter for q, the decision variable for the generalized position. */
  const VectorX<symbolic::Variable>& q_vars() const { return q_vars_; }

  /** Getter for u, the decision variable for the input. */
  const VectorX<symbolic::Variable>& u_vars() const { return u_vars_; }

  /**
   * Retrieve the solution to all contact wrenches.
   * @param result The result of solving prog().
   */
  std::vector<ContactWrench> GetContactWrenchSolution(
      const solvers::MathematicalProgramResult& result);

  /**
   * Updates the tolerance on all the complementarity constraints α * β = 0.
   * The complementarity constraint is relaxed as 0 ≤ α * β ≤ tol.
   * See AddStaticFrictionConeComplementarityConstraint() for more details.
   */
  void UpdateComplementarityTolerance(double tol);

 private:
  const MultibodyPlant<AutoDiffXd>& plant_;
  systems::Context<AutoDiffXd>* context_;
  std::unique_ptr<solvers::MathematicalProgram> owned_prog_;
  solvers::MathematicalProgram* prog_;
  VectorX<symbolic::Variable> q_vars_;
  VectorX<symbolic::Variable> u_vars_;

  std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                        VectorX<symbolic::Variable>>>
      contact_wrench_evaluators_and_lambda_;

  std::vector<solvers::Binding<
      internal::StaticFrictionConeComplementarityNonlinearConstraint>>
      static_friction_cone_complementarity_nonlinear_constraints_;
};
}  // namespace multibody
}  // namespace drake
