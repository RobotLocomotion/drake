#include "drake/multibody/contact_solvers/icf/icf_solver.h"

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double kConvergenceTolerance = 1e-8;

class IcfSolverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a simple model with two cliques.
    const int nv = 12;

    std::unique_ptr<IcfParameters<double>> params = model_.ReleaseParameters();
    params->time_step = 0.01;
    params->v0 = VectorXd::LinSpaced(nv, -1.0, 1.0);
    params->M0 = MatrixXd::Identity(nv, nv);
    params->M0.block<6, 6>(0, 0) = 0.9 * Matrix6<double>::Identity();
    params->M0.block<6, 6>(6, 6) = 1.7 * Matrix6<double>::Identity();
    params->D0 = VectorXd::Constant(nv, 0.1);
    params->k0 = VectorXd::LinSpaced(nv, -1.0, 1.0);
    params->clique_sizes = {6, 6};
    params->clique_start = {0, 6, nv};
    params->body_is_floating = {0, 0};
    params->body_mass = {0.2, 1.8};
    params->J_WB.Resize(2, 6, 6);
    params->J_WB[0] = VectorXd::LinSpaced(36, -1.0, 1.0).reshaped(6, 6);
    params->J_WB[1] = 3.4 * Matrix6<double>::Identity();
    params->body_to_clique = {0, 1};
    model_.ResetParameters(std::move(params));

    // Add a basic contact (patch) constraint with one pair and one patch.
    const double dissipation = 12.1;
    const double stiffness = 1.0e8;
    const double friction = 0.7;
    const Vector3<double> p_AB_W(-0.1, 0.3, -0.2);
    const Vector3<double> nhat_AB_W(0.0, 0.0, -1.0);
    const Vector3<double> p_BC_W = -0.5 * p_AB_W;
    const double fn0 = 10.5;

    PatchConstraintsPool<double>& patches = model_.patch_constraints_pool();
    const std::vector<int> num_pairs_per_patch = {1};
    patches.Resize(num_pairs_per_patch);
    patches.SetPatch(0 /* patch index */, 0 /* body A */, 1 /* body B */,
                     dissipation, friction, friction, p_AB_W);
    patches.SetPair(0 /* patch index */, 0 /* pair index */, p_BC_W, nhat_AB_W,
                    fn0, stiffness);

    // Make sure the data matches the model.
    model_.SetSparsityPattern();
    model_.ResizeData(&data_);

    // Set an arbitrary initial guess.
    data_.set_v(VectorXd::LinSpaced(nv, -0.5, 2.4));
  }

  IcfSolver solver_;
  IcfModel<double> model_;
  IcfData<double> data_;
};

/* For an unconstrained problem, the solver should converge in a single
iteration with no linesearch required. */
TEST_F(IcfSolverTest, UnconstrainedProblem) {
  // Remove patch constraints to recover an unconstrained problem.
  const std::vector<int> empty;
  model_.patch_constraints_pool().Resize(empty);
  model_.SetSparsityPattern();
  model_.ResizeData(&data_);

  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats& stats = solver_.stats();

  EXPECT_GT(stats.step_norm[0], 0.0);
  EXPECT_EQ(stats.num_iterations, 1);
  EXPECT_EQ(stats.ls_iterations[0], 0);
}

/* Solves a typical problem with patch constraints.*/
TEST_F(IcfSolverTest, Convergence) {
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats& stats = solver_.stats();

  // The problem should be difficult enough that it takes quite a few
  // iterations (and linesearch iterations) to converge.
  EXPECT_GE(stats.num_iterations, 5);
  const int total_ls_iterations = std::accumulate(stats.ls_iterations.begin(),
                                                  stats.ls_iterations.end(), 0);
  EXPECT_GE(total_ls_iterations, stats.num_iterations);

  // Check the sizes of the recorded stats.
  EXPECT_EQ(stats.cost.size(), stats.num_iterations);
  EXPECT_EQ(stats.gradient_norm.size(), stats.num_iterations);
  EXPECT_EQ(stats.ls_iterations.size(), stats.num_iterations);
  EXPECT_EQ(stats.alpha.size(), stats.num_iterations);
  EXPECT_EQ(stats.step_norm.size(), stats.num_iterations);

  // Cost and gradient norm should decrease at each iteration.
  for (int k = 1; k < stats.num_iterations; ++k) {
    EXPECT_LT(stats.cost[k], stats.cost[k - 1]);
    EXPECT_LT(stats.gradient_norm[k], stats.gradient_norm[k - 1]);
  }
}

/* Verifies that dense and sparse algebra produce the same result. */
TEST_F(IcfSolverTest, DenseVsSparseAlgebra) {
  const VectorXd v_guess = data_.v();

  // Solve with sparse algebra.
  EXPECT_FALSE(solver_.get_parameters().use_dense_algebra);
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats sparse_stats = solver_.stats();
  const VectorXd sparse_solution = data_.v();

  // Solve with dense algebra.
  IcfSolverParameters solver_params;
  solver_params.use_dense_algebra = true;
  solver_.SetParameters(solver_params);
  EXPECT_TRUE(solver_.get_parameters().use_dense_algebra);
  data_.set_v(v_guess);  // Reset the initial guess.
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats dense_stats = solver_.stats();
  const VectorXd dense_solution = data_.v();

  // Both approaches should take the same number of iterations.
  EXPECT_EQ(dense_stats.num_iterations, sparse_stats.num_iterations);

  // The solutions should be the same, up to the convergence tolerance.
  EXPECT_TRUE(CompareMatrices(dense_solution, sparse_solution,
                              kConvergenceTolerance,
                              MatrixCompareType::relative));
}

/* Verifies that Hessian reuse performs as expected. */
TEST_F(IcfSolverTest, HessianReuse) {
  const VectorXd v_guess = data_.v();
  IcfSolverParameters solver_params = solver_.get_parameters();

  // Solve without Hessian reuse.
  solver_params.enable_hessian_reuse = false;
  solver_.SetParameters(solver_params);
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats no_reuse_stats = solver_.stats();
  const VectorXd no_reuse_solution = data_.v();

  // Solve with Hessian reuse.
  solver_params.enable_hessian_reuse = true;
  solver_.SetParameters(solver_params);
  data_.set_v(v_guess);  // Reset the initial guess.
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats reuse_stats = solver_.stats();
  const VectorXd reuse_solution = data_.v();

  // Without reuse, we perform a factorization at every iteration.
  EXPECT_EQ(no_reuse_stats.num_factorizations, no_reuse_stats.num_iterations);

  // With reuse, we should perform fewer factorizations than iterations.
  // However, we should have forced at least two factorizations total: one at
  // the first time step, and at least one other when convergence stalled.
  EXPECT_LT(reuse_stats.num_factorizations, reuse_stats.num_iterations);
  EXPECT_GE(reuse_stats.num_factorizations, 2);

  // Solutions should be the same, up to the convergence tolerance used.
  EXPECT_TRUE(CompareMatrices(reuse_solution, no_reuse_solution,
                              kConvergenceTolerance,
                              MatrixCompareType::relative));
}

/* Verifies that no iterations are performed if the initial guess solves the
problem to the requested tolerance. */
TEST_F(IcfSolverTest, EarlyExit) {
  // Solve the problem once, writing the solution to data_.
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  EXPECT_GT(solver_.stats().num_iterations, 5);

  // Solve the problem again, with the solution as the initial guess (already
  // stored in data_).
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  EXPECT_EQ(solver_.stats().num_iterations, 0);
}

/* Verifies that the solver runs properly with printouts enabled. */
TEST_F(IcfSolverTest, PrintStatsSmokeTest) {
  IcfSolverParameters solver_params = solver_.get_parameters();
  solver_params.print_solver_stats = true;
  solver_.SetParameters(solver_params);
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
}

/* Checks that the solver works with several reasonable linesearch step size
limits. */
TEST_F(IcfSolverTest, LinesearchStepSizeLimits) {
  const VectorXd v_guess = data_.v();
  IcfSolverParameters solver_params = solver_.get_parameters();

  // Test several values for alpha_max.
  const std::vector<double> alpha_max_values = {0.5, 1.0, 1.5};
  for (const double alpha_max : alpha_max_values) {
    solver_params.alpha_max = alpha_max;
    solver_.SetParameters(solver_params);
    data_.set_v(v_guess);
    EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  }
}

/* Checks that we can converge to different tolerances. */
TEST_F(IcfSolverTest, ConvergenceTolerances) {
  const VectorXd v_guess = data_.v();

  // Solve with a very loose tolerance.
  EXPECT_TRUE(solver_.SolveWithGuess(model_, 1e-1, &data_));
  const int loose_tolerance_iterations = solver_.stats().num_iterations;

  // Solve with a tight tolerance, from the same initial guess.
  data_.set_v(v_guess);
  EXPECT_TRUE(solver_.SolveWithGuess(model_, 1e-6, &data_));
  const int tight_tolerance_iterations = solver_.stats().num_iterations;

  EXPECT_LT(loose_tolerance_iterations, tight_tolerance_iterations);
}

/* Verify that the solver returns false if it fails to converge. */
TEST_F(IcfSolverTest, ConvergenceFailure) {
  IcfSolverParameters solver_params = solver_.get_parameters();
  solver_params.max_iterations = 2;  // Too low for this problem.
  solver_.SetParameters(solver_params);
  EXPECT_FALSE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
}

/* Checks that the solver matches the analytical solution for a simple
unconstrained problem. */
TEST_F(IcfSolverTest, AnalyticalSolution) {
  // Solve a simple problem with a clear analytical solution.
  const std::vector<int> empty;
  model_.patch_constraints_pool().Resize(empty);
  model_.SetSparsityPattern();
  model_.ResizeData(&data_);
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const VectorXd solution = data_.v();

  // Compute the analytical solution, (M0 + h D0) (v - v0) + h k0 = 0.
  const MatrixXd M0 = model_.params().M0;
  const VectorXd D0 = model_.params().D0;
  const VectorXd v0 = model_.params().v0;
  const VectorXd k0 = model_.params().k0;
  const double h = model_.params().time_step;
  const MatrixXd A = M0 + h * D0.asDiagonal().toDenseMatrix();
  const VectorXd analytical_solution = v0 - A.ldlt().solve(h * k0);

  EXPECT_TRUE(CompareMatrices(solution, analytical_solution,
                              kConvergenceTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
