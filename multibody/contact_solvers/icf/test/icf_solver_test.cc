#include "drake/multibody/contact_solvers/icf/icf_solver.h"

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double kConvergenceTolerance = 1e-8;

class IcfSolverBaseTest : public ::testing::Test {
 protected:
  explicit IcfSolverBaseTest(bool use_islands) : use_islands_(use_islands) {
    auto params = solver_.get_parameters();
    params.use_islands = use_islands;
    solver_.SetParameters(params);
  }

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
    params->body_is_floating = {0, 0};
    params->body_mass = {0.2, 1.8};
    params->J_WB.Resize(2, 6, 6);
    params->J_WB[0] = VectorXd::LinSpaced(36, -1.0, 1.0).reshaped(6, 6);
    params->J_WB[1] = 3.4 * Matrix6<double>::Identity();
    params->body_to_clique = {0, 1};
    auto& reduction = params->reduction;
    reduction.unlocked_dofs = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    reduction.per_clique_unlocked_dofs = {
        {0, 1, 2, 3, 4, 5},
        {0, 1, 2, 3, 4, 5},
    };
    params->use_islands = use_islands_;
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

  bool use_islands_{};
  IcfSolver solver_;
  IcfModel<double> model_;
  IcfData<double> data_;
};

class IcfSolverParamTest : public IcfSolverBaseTest,
                           public ::testing::WithParamInterface<bool> {
 protected:
  IcfSolverParamTest() : IcfSolverBaseTest(/* use_islands = */ GetParam()) {}
};

/* For an unconstrained problem, the solver should converge in a single
iteration with no linesearch required. */
TEST_P(IcfSolverParamTest, UnconstrainedProblem) {
  // Remove patch constraints to recover an unconstrained problem.
  const std::vector<int> empty;
  model_.patch_constraints_pool().Resize(empty);
  model_.SetSparsityPattern();
  model_.ResizeData(&data_);

  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats& stats = solver_.stats();

  // Each island converges in a single iteration. With islands, the two cliques
  // are solved separately, so the total is one iteration per island.
  const int num_islands = use_islands_ ? 2 : 1;
  ASSERT_EQ(solver_.num_islands(), num_islands);
  EXPECT_EQ(stats.num_iterations, num_islands);
  EXPECT_EQ(stats.max_island_iterations, 1);
  for (int k = 0; k < num_islands; ++k) {
    EXPECT_GT(stats.step_norm[k], 0.0);
    EXPECT_EQ(stats.ls_iterations[k], 0);
  }
}

/* Solves a typical problem with patch constraints.*/
TEST_P(IcfSolverParamTest, Convergence) {
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
TEST_P(IcfSolverParamTest, DenseVsSparseAlgebra) {
  const VectorXd v_guess = data_.v();

  // Solve with sparse algebra.
  EXPECT_FALSE(solver_.get_parameters().use_dense_algebra);
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  const IcfSolverStats sparse_stats = solver_.stats();
  const VectorXd sparse_solution = data_.v();

  // Solve with dense algebra.
  IcfSolverParameters solver_params = solver_.get_parameters();
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
TEST_P(IcfSolverParamTest, HessianReuse) {
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
TEST_P(IcfSolverParamTest, EarlyExit) {
  // Solve the problem once, writing the solution to data_.
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  EXPECT_GT(solver_.stats().num_iterations, 5);

  // Solve the problem again, with the solution as the initial guess (already
  // stored in data_).
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  EXPECT_EQ(solver_.stats().num_iterations, 0);
}

/* Verifies that the solver runs properly with printouts enabled. */
TEST_P(IcfSolverParamTest, PrintStatsSmokeTest) {
  IcfSolverParameters solver_params = solver_.get_parameters();
  solver_params.print_solver_stats = true;
  solver_.SetParameters(solver_params);
  EXPECT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
}

/* Checks that the solver works with several reasonable linesearch step size
limits. */
TEST_P(IcfSolverParamTest, LinesearchStepSizeLimits) {
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
TEST_P(IcfSolverParamTest, ConvergenceTolerances) {
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
TEST_P(IcfSolverParamTest, ConvergenceFailure) {
  IcfSolverParameters solver_params = solver_.get_parameters();
  solver_params.max_iterations = 2;  // Too low for this problem.
  solver_.SetParameters(solver_params);
  EXPECT_FALSE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
}

/* Checks that the solver matches the analytical solution for a simple
unconstrained problem. */
TEST_P(IcfSolverParamTest, AnalyticalSolution) {
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

INSTANTIATE_TEST_SUITE_P(FlipIslands, IcfSolverParamTest, ::testing::Bool());

/* Makes a model whose cliques are coupled by constraints into two independent
islands (when use_islands is true), each with contact, so that both islands are
nonlinear. When `with_contact` is false, the patches are omitted and the model
splits into three (quadratic) islands. */
void MakeMultiIslandModel(IcfModel<double>* model, bool use_islands,
                          bool with_contact = true) {
  MakeUnconstrainedModel(model);
  auto params = model->ReleaseParameters();
  params->use_islands = use_islands;
  model->ResetParameters(std::move(params));
  AddCouplerConstraint(model);
  AddGainConstraints(model);
  AddLimitConstraints(model);
  if (with_contact) {
    AddPatchConstraints(model);
  }
  model->SetSparsityPattern();
}

/* Returns a solver with the given use_islands setting and otherwise default
parameters, modified by `edit`. */
std::unique_ptr<IcfSolver> MakeSolver(
    bool use_islands,
    const std::function<void(IcfSolverParameters*)>& edit = nullptr) {
  auto solver = std::make_unique<IcfSolver>();
  IcfSolverParameters params = solver->get_parameters();
  params.use_islands = use_islands;
  if (edit != nullptr) edit(&params);
  solver->SetParameters(params);
  return solver;
}

/* Solves `model` from `v_guess` with a fresh solver and data, returning v. */
VectorXd SolveFresh(const IcfModel<double>& model, const VectorXd& v_guess,
                    bool use_islands, double tolerance) {
  IcfData<double> data;
  model.ResizeData(&data);
  data.set_v(v_guess);
  std::unique_ptr<IcfSolver> solver = MakeSolver(use_islands);
  EXPECT_TRUE(solver->SolveWithGuess(model, tolerance, &data));
  return data.v();
}

/* With islands disabled (use_islands = false), the solver optimizes the full
problem across all cliques as a single Newton solve. On a multi-island problem
it must still reach the same minimizer as the island-decomposed solve. */
GTEST_TEST(IcfSolver, NoIslandsMatchesIslands) {
  std::vector<VectorXd> solutions{2};
  for (bool use_islands : {false, true}) {
    IcfModel<double> model;
    MakeMultiIslandModel(&model, use_islands);
    ASSERT_EQ(model.partition().num_islands(), use_islands ? 2 : 1);
    const VectorXd v_guess =
        VectorXd::LinSpaced(model.num_velocities(), -0.5, 0.7);
    solutions[use_islands] =
        SolveFresh(model, v_guess, use_islands, kConvergenceTolerance);
  }

  // Both reach the same minimizer, up to the convergence tolerance.
  EXPECT_TRUE(CompareMatrices(solutions[false], solutions[true],
                              kConvergenceTolerance,
                              MatrixCompareType::relative));
}

/* Returns ‖D⋅g‖ / max(1, ‖D⋅r‖) for the given island at `v`, where the norms
are restricted to the island's cliques. This is the island's convergence
measure, which the solver compares to the tolerance ε. */
double IslandScaledResidual(const IcfModel<double>& model, int island,
                            const VectorXd& v) {
  IcfData<double> data;
  model.ResizeData(&data);
  model.CalcData(v, island, &data);
  double g_sq = 0.0;
  double r_sq = 0.0;
  for (int c : model.partition().island_cliques(island)) {
    const auto D_c = model.clique_segment(c, model.scale_factor());
    g_sq += (D_c.array() * model.clique_segment(c, data.gradient()).array())
                .matrix()
                .squaredNorm();
    r_sq += (D_c.array() * model.clique_segment(c, model.r()).array())
                .matrix()
                .squaredNorm();
  }
  return std::sqrt(g_sq) / std::max(1.0, std::sqrt(r_sq));
}

/* Convergence is judged per island, against that island's own (normalized)
gradient and its own scale max(1, ‖D⋅r‖). In the regime where the absolute floor
applies to every island, the assembled residual can therefore exceed the
tolerance by up to a factor of sqrt(N) for N islands. This is intended: adding
independent copies of a problem must not require solving each copy more
accurately. */
GTEST_TEST(IcfSolver, ConvergenceIsPerIsland) {
  // Three independent cliques, with v₀ = 0 and k₀ = 0 so that r = 0 and every
  // island is in the absolute regime.
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  auto params = model.ReleaseParameters();
  params->use_islands = true;
  params->v0.setZero();
  params->k0.setZero();
  model.ResetParameters(std::move(params));
  model.SetSparsityPattern();
  const int num_islands = model.partition().num_islands();
  ASSERT_EQ(num_islands, 3);
  ASSERT_EQ(model.r().norm(), 0.0);
  const int nv = model.num_velocities();

  // Solve tightly for a reference solution.
  IcfData<double> data;
  model.ResizeData(&data);
  data.set_v(VectorXd::LinSpaced(nv, -0.5, 0.7));
  std::unique_ptr<IcfSolver> tight_solver =
      MakeSolver(true, [](IcfSolverParameters* p) {
        p->min_tolerance = 1e-12;
      });
  ASSERT_TRUE(tight_solver->SolveWithGuess(model, 1e-12, &data));
  const VectorXd v_star = data.v();

  // Offset each island from the solution by an amount that puts its own
  // convergence measure just inside the tolerance. The offsets are tiny, so the
  // gradient is locally linear and a single rescaling gets us there.
  const double tolerance = 1e-6;
  const double offset = 1e-6;
  const auto make_offset_guess = [&](const std::vector<double>& amplitudes) {
    VectorXd v = v_star;
    for (int i = 0; i < num_islands; ++i) {
      for (int c : model.partition().island_cliques(i)) {
        model.mutable_clique_segment(c, &v).array() += amplitudes[i];
      }
    }
    return v;
  };
  const VectorXd v_probe =
      make_offset_guess(std::vector<double>(num_islands, offset));
  std::vector<double> amplitudes;
  for (int i = 0; i < num_islands; ++i) {
    const double probe_residual = IslandScaledResidual(model, i, v_probe);
    ASSERT_GT(probe_residual, 0.0);
    amplitudes.push_back(offset * 0.9 * tolerance / probe_residual);
  }
  const VectorXd v_guess = make_offset_guess(amplitudes);

  // Every island is converged on its own, but the assembled residual exceeds ε
  // times the global scale, which would fail a global criterion.
  for (int i = 0; i < num_islands; ++i) {
    EXPECT_LT(IslandScaledResidual(model, i, v_guess), tolerance);
  }
  data.set_v(v_guess);
  for (int i = 0; i < num_islands; ++i) {
    model.CalcData(v_guess, i, &data);
  }
  const VectorXd Dg = model.scale_factor().array() * data.gradient().array();
  const VectorXd Dr = model.scale_factor().array() * model.r().array();
  EXPECT_GT(Dg.norm(), tolerance * std::max(1.0, Dr.norm()));

  // The solver accepts the guess without iterating on any island.
  std::unique_ptr<IcfSolver> solver = MakeSolver(true);
  model.ResizeData(&data);
  data.set_v(v_guess);
  EXPECT_TRUE(solver->SolveWithGuess(model, tolerance, &data));
  EXPECT_EQ(solver->stats().num_iterations, 0);
  EXPECT_EQ(data.v(), v_guess);
}

/* Fixture for nonlinear multi-island solves where one island starts at its
solution and the other is perturbed away from it. */
class IcfSolverPerturbedIslandTest : public ::testing::Test {
 protected:
  void SetUp() override {
    MakeMultiIslandModel(&model_, /* use_islands = */ true);
    ASSERT_EQ(model_.partition().num_islands(), 2);
    model_.ResizeData(&data_);

    // Solve tightly, so that the unperturbed island is converged at its
    // initial guess for the looser re-solves below.
    std::unique_ptr<IcfSolver> tight_solver =
        MakeSolver(true, [](IcfSolverParameters* p) {
          p->min_tolerance = 1e-12;
        });
    data_.set_v(VectorXd::LinSpaced(model_.num_velocities(), -0.5, 0.7));
    ASSERT_TRUE(tight_solver->SolveWithGuess(model_, 1e-12, &data_));

    // Perturb every clique of one island. The direction and size are chosen so
    // that the island's contact stays engaged, which makes it nonlinear (other
    // perturbations can leave it quadratic, solved in a single iteration).
    converged_island_ = model_.partition().clique_to_island(0);
    perturbed_island_ = 1 - converged_island_;
    v_guess_ = data_.v();
    for (int c : model_.partition().island_cliques(perturbed_island_)) {
      model_.mutable_clique_segment(c, &v_guess_).array() -= 1.0;
    }
    data_.set_v(v_guess_);
  }

  IcfModel<double> model_;
  IcfData<double> data_;
  VectorXd v_guess_;
  int converged_island_{-1};
  int perturbed_island_{-1};
};

/* Islands take different numbers of (nonlinear) iterations, one converging at
its initial guess, and the assembled solution matches the islands-off solve. */
TEST_F(IcfSolverPerturbedIslandTest, DifferentIterationCounts) {
  std::unique_ptr<IcfSolver> solver = MakeSolver(true);
  ASSERT_TRUE(solver->SolveWithGuess(model_, kConvergenceTolerance, &data_));
  EXPECT_EQ(solver->island_stats(converged_island_).num_iterations, 0);
  const int perturbed_iterations =
      solver->island_stats(perturbed_island_).num_iterations;
  EXPECT_GT(perturbed_iterations, 1);
  EXPECT_EQ(solver->stats().num_iterations, perturbed_iterations);

  // The islands-off solve of the same problem (one island holding all cliques)
  // reaches the same minimizer.
  IcfModel<double> model_off;
  MakeMultiIslandModel(&model_off, /* use_islands = */ false);
  ASSERT_EQ(model_off.partition().num_islands(), 1);
  const VectorXd v_off =
      SolveFresh(model_off, v_guess_, false, kConvergenceTolerance);
  EXPECT_TRUE(CompareMatrices(data_.v(), v_off, kConvergenceTolerance,
                              MatrixCompareType::relative));
}

/* When one island fails to converge, the solve reports failure, while the
other (already converged) island's velocities are left exactly as they were. */
TEST_F(IcfSolverPerturbedIslandTest, OneIslandFails) {
  std::unique_ptr<IcfSolver> solver =
      MakeSolver(true, [](IcfSolverParameters* p) {
        p->max_iterations = 1;
      });
  EXPECT_FALSE(solver->SolveWithGuess(model_, kConvergenceTolerance, &data_));
  EXPECT_EQ(solver->island_stats(converged_island_).num_iterations, 0);
  EXPECT_EQ(solver->island_stats(perturbed_island_).num_iterations, 1);
  for (int c : model_.partition().island_cliques(converged_island_)) {
    EXPECT_EQ(model_.clique_segment(c, data_.v()),
              model_.clique_segment(c, v_guess_));
  }
}

/* A single solver and data, with Hessian reuse, stay correct while constraints
merge islands and later split them again. */
GTEST_TEST(IcfSolver, ReuseAcrossIslandMergeAndSplit) {
  std::unique_ptr<IcfSolver> solver =
      MakeSolver(true, [](IcfSolverParameters* p) {
        p->enable_hessian_reuse = true;
      });
  IcfData<double> data;
  for (bool with_contact : {false, true, true, false}) {
    SCOPED_TRACE(fmt::format("with_contact = {}", with_contact));
    IcfModel<double> model;
    MakeMultiIslandModel(&model, /* use_islands = */ true, with_contact);
    ASSERT_EQ(model.partition().num_islands(), with_contact ? 2 : 3);
    const VectorXd v_guess =
        VectorXd::LinSpaced(model.num_velocities(), -0.5, 0.7);

    model.ResizeData(&data);
    data.set_v(v_guess);
    ASSERT_TRUE(solver->SolveWithGuess(model, kConvergenceTolerance, &data));
    EXPECT_EQ(solver->num_islands(), model.partition().num_islands());

    const VectorXd v_fresh =
        SolveFresh(model, v_guess, true, kConvergenceTolerance);
    EXPECT_TRUE(CompareMatrices(data.v(), v_fresh, kConvergenceTolerance,
                                MatrixCompareType::relative));
  }
}

/* Checks the aggregation of per-island statistics when the islands take
different numbers of iterations, including an island that converges at its
initial guess. */
class IcfSolverIslandStatsTest : public IcfSolverBaseTest {
 protected:
  IcfSolverIslandStatsTest() : IcfSolverBaseTest(/* use_islands = */ true) {}
};

TEST_F(IcfSolverIslandStatsTest, DifferentIterationCounts) {
  // Remove patch constraints, leaving two independent (quadratic) islands.
  const std::vector<int> empty;
  model_.patch_constraints_pool().Resize(empty);
  model_.SetSparsityPattern();
  model_.ResizeData(&data_);
  ASSERT_EQ(model_.partition().num_islands(), 2);

  // Find the solution, then perturb only the second clique's velocities so
  // that its island must iterate while the other is already converged.
  ASSERT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));
  VectorXd v_guess = data_.v();
  model_.mutable_clique_segment(1, &v_guess).array() += 0.3;
  data_.set_v(v_guess);
  ASSERT_TRUE(solver_.SolveWithGuess(model_, kConvergenceTolerance, &data_));

  ASSERT_EQ(solver_.num_islands(), 2);
  const int converged_island = model_.partition().clique_to_island(0);
  const int perturbed_island = model_.partition().clique_to_island(1);
  const IcfSolverStats& converged = solver_.island_stats(converged_island);
  const IcfSolverStats& perturbed = solver_.island_stats(perturbed_island);
  EXPECT_EQ(converged.num_iterations, 0);
  EXPECT_TRUE(converged.cost.empty());
  EXPECT_EQ(perturbed.num_iterations, 1);
  EXPECT_EQ(perturbed.max_island_iterations, 1);

  // The aggregate stats are totals over islands, and every per-iteration
  // history is the concatenation of the islands' histories.
  const IcfSolverStats& stats = solver_.stats();
  EXPECT_EQ(stats.num_iterations, 1);
  EXPECT_EQ(stats.max_island_iterations, 1);
  EXPECT_EQ(stats.num_factorizations,
            converged.num_factorizations + perturbed.num_factorizations);
  EXPECT_EQ(stats.cost, perturbed.cost);
  EXPECT_EQ(stats.gradient_norm.size(), stats.num_iterations);
  EXPECT_EQ(stats.ls_iterations.size(), stats.num_iterations);
  EXPECT_EQ(stats.alpha.size(), stats.num_iterations);
  EXPECT_EQ(stats.step_norm.size(), stats.num_iterations);
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
