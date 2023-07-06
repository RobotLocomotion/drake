#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/sap/sap_friction_cone_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Tests that SolveWithGuess returns the free motion velocities and zero
// impulses if the constraints are empty and throws otherwise.
GTEST_TEST(SapAutoDiffTest, SolveWithGuess) {
  constexpr double kTimeStep = 0.001;
  constexpr int kNumDofs = 5;
  constexpr int kNumCliques = 1;
  std::vector<MatrixX<AutoDiffXd>> A(kNumCliques);
  A[0] = MatrixX<AutoDiffXd>::Identity(kNumDofs, kNumDofs);
  VectorX<AutoDiffXd> v_star =
      VectorX<AutoDiffXd>::LinSpaced(kNumDofs, 0.0, 1.0);
  SapSolver<AutoDiffXd> sap;

  // The case without constraints.
  SapContactProblem<AutoDiffXd> contact_problem_without_constraint(
      kTimeStep, std::move(A), v_star);
  // Use NaN as the guess as it shouldn't be used at all in the early exit where
  // there's no constraint.
  const VectorX<AutoDiffXd> v_guess =
      VectorX<AutoDiffXd>::Constant(kNumDofs, NAN);
  SapSolverResults<AutoDiffXd> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(contact_problem_without_constraint, v_guess, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);
  EXPECT_EQ(result.v, v_star);
  EXPECT_EQ(result.j, VectorX<AutoDiffXd>::Zero(kNumDofs));

  // The case with constraints.
  constexpr int kCliqueIndex = 0;
  constexpr int kNumConstraintEquations = 3;
  // Dummy parameter to create an arbitrary constraint.
  constexpr double kPhi0 = 0.1;
  SapFrictionConeConstraint<AutoDiffXd>::Parameters parameters;
  parameters.stiffness = 1.0;
  SapConstraintJacobian<AutoDiffXd> J(
      kCliqueIndex,
      MatrixX<AutoDiffXd>::Ones(kNumConstraintEquations, kNumDofs));
  SapContactProblem<AutoDiffXd> contact_problem_with_constraint(
      std::move(contact_problem_without_constraint));
  // Since a contact constraint is involved, there must be at least one object
  // with a valid index, equal to zero.
  contact_problem_with_constraint.set_num_objects(1);
  // For this test, only the signed distance phi is relevant.
  // Object indices must be valid, even though not used in these tests. Every
  // other configuration will be left uninitialized.
  ContactConfiguration<AutoDiffXd> configuration{
      .objectA = 0 /* valid, though not used */,
      .objectB = 0 /* valid, though not used */,
      .phi = kPhi0};
  contact_problem_with_constraint.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<AutoDiffXd>>(
          std::move(configuration), std::move(J), std::move(parameters)));
  DRAKE_EXPECT_THROWS_MESSAGE(
      sap.SolveWithGuess(contact_problem_with_constraint, v_guess, &result),
      ".*Only.*double is supported.*");
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
