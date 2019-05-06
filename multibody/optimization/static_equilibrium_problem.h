#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "drake/multibody/optimization/contact_wrench.h"
#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/multibody/optimization/static_friction_cone_complementary_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
class StaticEquilibriumProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticEquilibriumProblem)

  /**
   * @param plant The plant for which the static equilibrium posture is
   * computed.
   * @param context The context for `plant`.
   * @param ignored_collision_pairs The contact between the pair of geometry in
   * `ignored_collision_pairs` will be ignored. We will not impose
   * non-penetration constraint between these pairs, and no contact wrench will
   * be implied between these pairs.
   */
  StaticEquilibriumProblem(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>&
          ignored_collision_pairs);

  ~StaticEquilibriumProblem() {}

  /** Getter for the mutable optimization program. */
  solvers::MathematicalProgram* get_mutable_prog() const { return prog_; }

  /** Getter for the immutable optimization program. */
  const solvers::MathematicalProgram& prog() const { return *prog_; }

  /** Getter for q, the decision variable for the generalized position. */
  const VectorX<symbolic::Variable>& q_vars() const { return q_vars_; }

  /** Getter for u, the decision variable for the input. */
  const VectorX<symbolic::Variable>& u_vars() const { return u_vars_; }

  /**
   * Retrive the solution to all contact wrenches.
   * @param result The result of solving prog().
   */
  std::vector<ContactWrench> GetContactWrenchSolution(
      const solvers::MathematicalProgramResult& result);

  /**
   * Updates the tolerance on all the complemntary constraints.
   * The complementary constraint is relaxed as 0 ≤ α * β ≤ tol.
   */
  void UpdateComplementaryTolerance(double tol);

 private:
  const MultibodyPlant<AutoDiffXd>* const plant_;
  systems::Context<AutoDiffXd>* context_;
  std::unique_ptr<solvers::MathematicalProgram> owned_prog_;
  solvers::MathematicalProgram* prog_;
  VectorX<symbolic::Variable> q_vars_;
  VectorX<symbolic::Variable> u_vars_;

  std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                        VectorX<symbolic::Variable>>>
      contact_wrench_evaluators_and_lambda_;

  std::vector<solvers::Binding<
      internal::StaticFrictionConeComplementaryNonlinearConstraint>>
      static_friction_cone_complementary_nonlinear_constraints_;
};
}  // namespace multibody
}  // namespace drake
