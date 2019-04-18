#pragma once

#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
class StaticEquilibriumSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticEquilibriumSolver)

  StaticEquilibriumSolver(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>&
          ignored_collision_pairs);

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
};
}  // namespace multibody
}  // namespace drake
