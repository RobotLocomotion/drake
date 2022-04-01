#include "drake/multibody/fem/fem_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/acceleration_newmark_scheme.h"
#include "drake/multibody/fem/test/dummy_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::MatrixXd;
constexpr double kEps = 4.0 * std::numeric_limits<double>::epsilon();
/* Parameters for the Newmark-beta integration scheme. */
constexpr double kDt = 0.01;
constexpr double kGamma = 0.5;
constexpr double kBeta = 0.25;

class FemSolverTest : public ::testing::Test {
 protected:
  DummyModel model_{};
  AccelerationNewmarkScheme<double> integrator_{kDt, kGamma, kBeta};
  FemSolver<double> solver_{&model_, &integrator_};
};

TEST_F(FemSolverTest, Tolerancse) {
  /* Default values. */
  EXPECT_EQ(solver_.relative_tolerance(), 1e-4);
  EXPECT_EQ(solver_.absolute_tolerance(), 1e-6);
  /* Test Setters. */
  constexpr double kTol = 1e-8;
  solver_.set_relative_tolerance(kTol);
  solver_.set_absolute_tolerance(kTol);
  EXPECT_EQ(solver_.relative_tolerance(), kTol);
  EXPECT_EQ(solver_.absolute_tolerance(), kTol);
}

/* Tests that the behavior of FemSolver::AdvanceOneTimeStep agrees with analytic
 results. The DummyModel contains DummyElements that give non-zero residual at
 zero state and zero residual everywhere else. The dummy element also has
 constant stiffness, damping, and mass matrix (aka they are state-independent).
 As a result, we expect AdvanceOneTimeStep to converge after exactly one Newton
 iteration, and the unknown variable z (acceleration in this case) should
 satisfy A*z = -b, where A is the constant tangent matrix and b is the nonzero
 residual evaluated at the zero state. */
TEST_F(FemSolverTest, AdvanceOneTimeStep) {
  DummyModel::DummyBuilder builder(&model_);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  std::unique_ptr<FemState<double>> state0 = model_.MakeFemState();
  std::unique_ptr<FemState<double>> state = model_.MakeFemState();
  const int num_iterations = solver_.AdvanceOneTimeStep(*state0, state.get());
  EXPECT_EQ(num_iterations, 1);

  /* Compute the expected result from AdvanceOneTimeStep(). */
  std::unique_ptr<FemState<double>> expected_state = model_.MakeFemState();
  auto tangent_matrix = model_.MakePetscSymmetricBlockSparseTangentMatrix();
  model_.CalcTangentMatrix(*state0, integrator_.GetWeights(),
                           tangent_matrix.get());
  tangent_matrix->AssembleIfNecessary();
  const MatrixXd A = tangent_matrix->MakeDenseMatrix();
  VectorX<double> b(model_.num_dofs());
  model_.CalcResidual(*state0, &b);
  /* The solver is not considered as "converged" with the initial state. */
  EXPECT_FALSE(solver_.solver_converged(b.norm(), b.norm()));
  Eigen::ConjugateGradient<MatrixXd> cg;
  cg.compute(A);
  const VectorX<double> dz = cg.solve(-b);
  integrator_.UpdateStateFromChangeInUnknowns(dz, expected_state.get());
  EXPECT_TRUE(CompareMatrices(expected_state->GetPositions(),
                              state->GetPositions(), kEps));
  EXPECT_TRUE(CompareMatrices(expected_state->GetAccelerations(),
                              state->GetAccelerations(), kEps));
  EXPECT_TRUE(CompareMatrices(expected_state->GetVelocities(),
                              state->GetVelocities(), kEps));
}

// TODO(xuchenhan-tri): Unit tests that cover other exit conditions of
// the iterative solver are missing.

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
