#include "drake/multibody/contact_solvers/icf/icf_model.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_search_direction_data.h"
#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

/* Checks that a default constructed model is empty. */
GTEST_TEST(IcfModel, EmptyModel) {
  IcfModel<double> model;
  EXPECT_EQ(model.num_bodies(), 0);
  EXPECT_EQ(model.num_cliques(), 0);
  EXPECT_EQ(model.num_velocities(), 0);
  EXPECT_EQ(model.num_constraints(), 0);
  EXPECT_EQ(model.max_clique_size(), 0);

  EXPECT_EQ(model.M0().rows(), 0);
  EXPECT_EQ(model.M0().cols(), 0);
  EXPECT_EQ(model.v0().size(), 0);
  EXPECT_EQ(model.D0().size(), 0);
  EXPECT_EQ(model.k0().size(), 0);
  EXPECT_EQ(model.scale_factor().size(), 0);
  EXPECT_EQ(model.r().size(), 0);
}

/* Checks that an empty model is constructed with minimal heap allocations. */
GTEST_TEST(IcfModel, LimitMallocOnModelUpdate) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);  // Memory is allocated here.
  EXPECT_EQ(model.num_bodies(), 4);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 0);
  EXPECT_EQ(model.max_clique_size(), 6);

  EXPECT_EQ(model.M0().rows(), 18);
  EXPECT_EQ(model.M0().cols(), 18);
  EXPECT_EQ(model.v0().size(), 18);
  EXPECT_EQ(model.D0().size(), 18);
  EXPECT_EQ(model.k0().size(), 18);
  EXPECT_EQ(model.scale_factor().size(), 18);
  EXPECT_EQ(model.r().size(), 18);
  EXPECT_EQ(model.A(0).rows(), 6);
  EXPECT_EQ(model.A(0).cols(), 6);

  // Updating on the same size model should cause no new allocations.
  {
    drake::test::LimitMalloc guard;
    MakeUnconstrainedModel(&model);
  }
}

/* Iterates over each body to check sizes and such. */
GTEST_TEST(IcfModel, PerBodyElements) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  EXPECT_EQ(model.num_bodies(), 4);
  EXPECT_EQ(model.num_velocities(), 18);

  const VectorXd& v = model.v0();

  // We'll fill this with ones, clique by clique.
  VectorXd mutable_vector = VectorXd::Zero(model.num_velocities());

  int num_anchored = 0;
  int num_floating = 0;
  for (int b = 0; b < model.num_bodies(); ++b) {
    const Vector6d& V_WB = model.V_WB0(b);
    const int c = model.body_to_clique(b);
    const int clique_nv = model.clique_size(c);

    if (model.is_anchored(b)) {
      ++num_anchored;
      EXPECT_LT(c, 0);
      EXPECT_EQ(clique_nv, 0);

      // Spatial velocity should be zero.
      EXPECT_TRUE(CompareMatrices(V_WB, Vector6d::Zero(), kEpsilon,
                                  MatrixCompareType::relative));
    } else if (model.is_floating(b)) {
      ++num_floating;
      EXPECT_GE(c, 0);
      EXPECT_EQ(clique_nv, 6);

      // Spatial velocity should equal v0 segment.
      const Vector6d& V_WB_expected = model.clique_segment(c, v);
      EXPECT_TRUE(CompareMatrices(V_WB, V_WB_expected, kEpsilon,
                                  MatrixCompareType::relative));

    } else {
      EXPECT_GE(c, 0);

      // Spatial velocity should equal J_WB * v0 segment.
      const Matrix6X<double>& J_WB = model.J_WB(b);
      EXPECT_EQ(J_WB.cols(), clique_nv);
      const Vector6d V_WB_expected = J_WB * model.clique_segment(c, v);
      EXPECT_TRUE(CompareMatrices(V_WB, V_WB_expected, kEpsilon,
                                  MatrixCompareType::relative));
    }

    EXPECT_GT(model.body_mass(b), 0.0);
    if (!model.is_anchored(b)) {
      EXPECT_GT(model.clique_diagonal_mass_inverse(c).minCoeff(), 0.0);
      model.mutable_clique_segment(c, &mutable_vector) +=
          VectorXd::Ones(clique_nv);
    }
  }

  // Our mutable vector should now be filled with ones in each clique segment.
  // Any zeros would indicate that a segment was missed, while twos would
  // indicate that a segment was double counted.
  EXPECT_TRUE(CompareMatrices(mutable_vector,
                              VectorXd::Ones(model.num_velocities()), kEpsilon,
                              MatrixCompareType::relative));

  // We should have exactly one floating body and one anchored body.
  EXPECT_EQ(num_floating, 1);
  EXPECT_EQ(num_anchored, 1);
}

/* Checks that gradients are computed correctly, even in the presence of
constraints. */
GTEST_TEST(IcfModel, CalcGradients) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model);
  AddCouplerConstraint(&model);
  AddGainConstraints(&model);
  AddLimitConstraints(&model);
  AddPatchConstraints(&model);
  AddWeldConstraints(&model);
  model.SetSparsityPattern();
  const int nv = model.num_velocities();

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), nv);
  EXPECT_EQ(model.num_constraints(), 10);

  VectorXd v_values = VectorXd::LinSpaced(nv, -10.0, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);

  const VectorXd cost_derivatives = data.cost().derivatives();
  const VectorXd gradient_value = math::ExtractValue(data.gradient());

  EXPECT_TRUE(CompareMatrices(gradient_value, cost_derivatives, 100 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Checks the Hessian for a problem with a single clique. */
GTEST_TEST(IcfModel, CalcDenseHessian) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model, true /* single cliques */);
  AddCouplerConstraint(&model);
  AddGainConstraints(&model);
  AddLimitConstraints(&model);
  AddPatchConstraints(&model);
  AddWeldConstraints(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 1);
  EXPECT_EQ(model.num_velocities(), 18);
  const int nv = model.num_velocities();

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());

  VectorXd v_values = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_values, &v);

  model.CalcData(v, &data);
  MatrixXd gradient_derivatives = math::ExtractGradient(data.gradient());

  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 8 * kEpsilon,
                              MatrixCompareType::relative));

  // Now we'll only update the values.
  v_values = VectorXd::LinSpaced(nv, -3.0, 7.0);
  math::InitializeAutoDiff(v_values, &v);
  model.CalcData(v, &data);
  model.UpdateHessian(data, hessian.get());

  gradient_derivatives = math::ExtractGradient(data.gradient());
  hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());

  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives, 8 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Checks that we get the same result for the same problem, regardless of how we
break out the cliques. */
GTEST_TEST(IcfModel, SingleVsMultipleCliques) {
  IcfModel<double> model_single;
  MakeUnconstrainedModel(&model_single, true);
  AddCouplerConstraint(&model_single);
  AddGainConstraints(&model_single);
  AddLimitConstraints(&model_single);
  AddPatchConstraints(&model_single);
  AddWeldConstraints(&model_single);
  model_single.SetSparsityPattern();
  EXPECT_EQ(model_single.num_cliques(), 1);
  EXPECT_EQ(model_single.num_velocities(), 18);

  IcfModel<double> model_multiple;
  MakeUnconstrainedModel(&model_multiple, false);
  AddCouplerConstraint(&model_multiple);
  AddGainConstraints(&model_multiple);
  AddLimitConstraints(&model_multiple);
  AddPatchConstraints(&model_multiple);
  AddWeldConstraints(&model_multiple);
  model_multiple.SetSparsityPattern();
  EXPECT_EQ(model_multiple.num_cliques(), 3);
  EXPECT_EQ(model_multiple.num_velocities(), 18);

  // Allocate data.
  IcfData<double> data_single;
  model_single.ResizeData(&data_single);
  IcfData<double> data_multiple;
  model_multiple.ResizeData(&data_multiple);

  // Compute data.
  const int nv = model_single.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model_single.CalcData(v, &data_single);
  model_multiple.CalcData(v, &data_multiple);

  // Compute Hessians.
  auto hessian_single = model_single.MakeHessian(data_single);
  auto hessian_multiple = model_multiple.MakeHessian(data_multiple);
  MatrixXd H_single = hessian_single->MakeDenseMatrix();
  MatrixXd H_multiple = hessian_multiple->MakeDenseMatrix();

  // Verify sparsity patterns.
  EXPECT_EQ(hessian_single->block_rows(), 1);
  EXPECT_EQ(hessian_single->block_cols(), 1);
  EXPECT_EQ(hessian_multiple->block_rows(), 3);
  EXPECT_EQ(hessian_multiple->block_cols(), 3);

  EXPECT_EQ(model_single.sparsity_pattern().block_sizes(),
            std::vector<int>({18}));
  EXPECT_EQ(model_multiple.sparsity_pattern().block_sizes(),
            std::vector<int>({6, 6, 6}));

  // Cost, gradient, and Hessian should be the same regardless of sparsity.
  // We use 100 * kEpsilon because cross-clique weld constraints accumulate
  // slightly different round-off when the same problem is decomposed into one
  // vs. three cliques.
  EXPECT_NEAR(data_single.cost(), data_multiple.cost(), 100 * kEpsilon);
  EXPECT_TRUE(CompareMatrices(data_single.gradient(), data_multiple.gradient(),
                              100 * kEpsilon, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(H_single, H_multiple, 100 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Checks that our exact linesearch computations are correct. */
GTEST_TEST(IcfModel, CalcCostAlongLine) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model);
  AddCouplerConstraint(&model);
  AddGainConstraints(&model);
  AddLimitConstraints(&model);
  AddPatchConstraints(&model);
  AddWeldConstraints(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 10);

  // Allocate data, and additional scratch.
  IcfData<AutoDiffXd> data, scratch;
  model.ResizeData(&data);
  model.ResizeData(&scratch);

  // Compute data.
  const int nv = model.num_velocities();
  const VectorX<AutoDiffXd> v = VectorXd::LinSpaced(nv, 0.05, 0.01);
  model.CalcData(v, &data);

  // Allocate search direction.
  const VectorX<AutoDiffXd> w = VectorX<AutoDiffXd>::LinSpaced(
      nv, 0.1, -0.2);  // Arbitrary search direction.
  IcfSearchDirectionData<AutoDiffXd> search_data;
  model.CalcSearchDirectionData(data, w, &search_data);

  // Try-out a set of arbitrary values.
  for (double alpha_value : {-0.45, 0., 0.15, 0.34, 0.93, 1.32}) {
    const AutoDiffXd alpha = {
        alpha_value, VectorXd::Ones(1) /* This is the independent variable */};

    const VectorX<AutoDiffXd> v_alpha = v + alpha * w;
    model.CalcData(v_alpha, &scratch);
    const double cost_expected = scratch.cost().value();
    const double momentum_cost_expected = scratch.momentum_cost().value();
    const double dcost_expected = scratch.cost().derivatives()[0];
    const VectorXd w_times_H = math::ExtractGradient(scratch.gradient());
    const double d2cost_expected = w_times_H.dot(math::ExtractValue(w));

    // Verify pre-computed terms are correct.
    const AutoDiffXd a = search_data.a;
    const AutoDiffXd b = search_data.b;
    const AutoDiffXd c = search_data.c;
    const double momentum_cost =
        (a * alpha * alpha / 2.0 + b * alpha + c).value();
    EXPECT_NEAR(momentum_cost, momentum_cost_expected,
                8 * kEpsilon * abs(momentum_cost_expected));

    AutoDiffXd dcost, d2cost;
    const AutoDiffXd cost =
        model.CalcCostAlongLine(alpha, data, search_data, &dcost, &d2cost);
    EXPECT_NEAR(cost.value(), cost_expected, 8 * kEpsilon * abs(cost_expected));
    EXPECT_NEAR(dcost.value(), dcost_expected,
                8 * kEpsilon * abs(dcost_expected));
    EXPECT_NEAR(d2cost.value(), d2cost_expected,
                8 * kEpsilon * abs(d2cost_expected));
  }
}

/* Checks that updating the time step produces the same result as creating a new
model from scratch. */
GTEST_TEST(IcfModel, UpdateTimeStep) {
  IcfModel<double> model_original;
  MakeUnconstrainedModel(&model_original, false, 0.02);
  AddCouplerConstraint(&model_original);
  AddGainConstraints(&model_original);
  AddLimitConstraints(&model_original);
  AddPatchConstraints(&model_original);
  AddWeldConstraints(&model_original);
  model_original.SetSparsityPattern();
  EXPECT_EQ(model_original.num_cliques(), 3);
  EXPECT_EQ(model_original.num_velocities(), 18);
  EXPECT_EQ(model_original.time_step(), 0.02);
  EXPECT_EQ(model_original.num_constraints(), 10);

  const double new_time_step = 0.003;

  // Create a second model from scratch with the new time step.
  IcfModel<double> model_new;
  MakeUnconstrainedModel(&model_new, false, new_time_step);
  AddCouplerConstraint(&model_new);
  AddGainConstraints(&model_new);
  AddLimitConstraints(&model_new);
  AddPatchConstraints(&model_new);
  AddWeldConstraints(&model_new);
  model_new.SetSparsityPattern();
  EXPECT_EQ(model_new.num_cliques(), 3);
  EXPECT_EQ(model_new.num_velocities(), 18);
  EXPECT_EQ(model_new.time_step(), new_time_step);
  EXPECT_EQ(model_new.num_constraints(), 10);

  // Now update the time step of the original model.
  EXPECT_NE(model_original.time_step(), new_time_step);
  model_original.UpdateTimeStep(new_time_step);
  EXPECT_EQ(model_original.time_step(), new_time_step);

  // Allocate data.
  IcfData<double> data_original, data_new;
  model_original.ResizeData(&data_original);
  model_new.ResizeData(&data_new);

  // Compute data for an arbitrary velocity.
  const int nv = model_original.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model_original.CalcData(v, &data_original);
  model_new.CalcData(v, &data_new);

  // Cost and gradient should be the same.
  EXPECT_NEAR(data_original.cost(), data_new.cost(), 8 * kEpsilon);
  EXPECT_TRUE(CompareMatrices(data_original.gradient(), data_new.gradient(),
                              8 * kEpsilon, MatrixCompareType::relative));

  // Hessians should be the same.
  auto hessian_original = model_original.MakeHessian(data_original);
  auto hessian_new = model_new.MakeHessian(data_new);
  MatrixXd H_original = hessian_original->MakeDenseMatrix();
  MatrixXd H_new = hessian_new->MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(H_original, H_new, 8 * kEpsilon,
                              MatrixCompareType::relative));
}

/* Verifies that params() and ReleaseParams() use the same address. */
GTEST_TEST(IcfModel, ParamsAccessors) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);

  const IcfParameters<double>* params1 = &model.params();
  std::unique_ptr<IcfParameters<double>> params2 = model.ReleaseParameters();
  EXPECT_EQ(params1, params2.get());
}

/* Collects an IslandItemMap into a vector-of-vectors for easy comparison. */
std::vector<std::vector<int>> ToVectors(const IslandItemMap& map) {
  std::vector<std::vector<int>> result(map.num_islands());
  for (int i = 0; i < map.num_islands(); ++i) {
    const auto items = map.items(i);
    result[i].assign(items.begin(), items.end());
  }
  return result;
}

/* With no constraints, every clique is its own island. The three dynamic bodies
(1, 2, 3) land in cliques 0, 1, 2 respectively; the world (body 0) is anchored
and belongs to no island. */
GTEST_TEST(IcfModel, IslandsUnconstrained) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();

  const IcfPartition& partition = model.partition();
  EXPECT_EQ(partition.num_islands(), 3);
  EXPECT_EQ(ToVectors(model.island_bodies()),
            (std::vector<std::vector<int>>{{1}, {2}, {3}}));
}

/* Patches couple cliques 0 and 1 (patches 0 and 2), while patch 1 touches only
clique 2 (it is anchored to the world). This yields two islands: {0, 1} and {2}.
The bodies and patch constraints partition accordingly. */
GTEST_TEST(IcfModel, IslandsWithPatchesOnly) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddPatchConstraints(&model);
  model.SetSparsityPattern();

  const IcfPartition& partition = model.partition();
  ASSERT_EQ(partition.num_islands(), 2);
  EXPECT_EQ(partition.clique_to_island(0), 0);
  EXPECT_EQ(partition.clique_to_island(1), 0);
  EXPECT_EQ(partition.clique_to_island(2), 1);

  // Bodies 1 (clique 0) and 2 (clique 1) are in island 0; body 3 (clique 2) is
  // in island 1.
  EXPECT_EQ(ToVectors(model.island_bodies()),
            (std::vector<std::vector<int>>{{1, 2}, {3}}));
  // Patches 0 and 2 couple cliques {0, 1} (island 0); patch 1 is on clique 2
  // (island 1).
  EXPECT_EQ(ToVectors(model.island_patches()),
            (std::vector<std::vector<int>>{{0, 2}, {1}}));
}

/* Adding the welds connects clique 1 to clique 2 (weld 1), merging everything
into a single island. All constraint kinds then map to island 0. */
GTEST_TEST(IcfModel, IslandsFullyCoupled) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddCouplerConstraint(&model);
  AddGainConstraints(&model);
  AddLimitConstraints(&model);
  AddPatchConstraints(&model);
  AddWeldConstraints(&model);
  model.SetSparsityPattern();

  ASSERT_EQ(model.partition().num_islands(), 1);
  // Every constraint and dynamic body belongs to the single island 0.
  EXPECT_EQ(ToVectors(model.island_bodies()),
            (std::vector<std::vector<int>>{{1, 2, 3}}));
  EXPECT_EQ(ToVectors(model.island_couplers()),
            (std::vector<std::vector<int>>{{0}}));
  EXPECT_EQ(ToVectors(model.island_gains()),
            (std::vector<std::vector<int>>{{0, 1}}));
  EXPECT_EQ(ToVectors(model.island_limits()),
            (std::vector<std::vector<int>>{{0, 1}}));
  EXPECT_EQ(ToVectors(model.island_patches()),
            (std::vector<std::vector<int>>{{0, 1, 2}}));
  EXPECT_EQ(ToVectors(model.island_welds()),
            (std::vector<std::vector<int>>{{0, 1}}));
}

/* Verifies that evaluating a model island-by-island reproduces the full-problem
cost, gradient, Hessian, and cost-along-line. This is the core Phase 2b
invariant: ℓ = Σᵢ ℓᵢ, the gradient is block-disjoint across islands, and the
Hessian is block-diagonal across islands. */
void CheckIslandEquivalence(const IcfModel<double>& model, const VectorXd& v) {
  const int nv = model.num_velocities();
  const IcfPartition& partition = model.partition();
  const int num_islands = partition.num_islands();
  ASSERT_GT(num_islands, 0);

  // Full-problem evaluation.
  IcfData<double> data_full;
  model.ResizeData(&data_full);
  model.CalcData(v, &data_full);
  const MatrixXd H_full = model.MakeHessian(data_full)->MakeDenseMatrix();

  // Island-by-island evaluation. The caller (here) sets v once; each island
  // then writes only its own segments of the shared data.
  IcfData<double> data_isl;
  model.ResizeData(&data_isl);
  data_isl.set_v(v);
  double island_cost_sum = 0.0;
  MatrixXd H_islands = MatrixXd::Zero(nv, nv);
  for (int i = 0; i < num_islands; ++i) {
    const double cost_i = model.CalcData(v, i, &data_isl);
    EXPECT_EQ(cost_i, data_isl.island_cost(i));
    island_cost_sum += cost_i;

    // Scatter the island's local sub-Hessian into the global velocity indices
    // of its cliques.
    const MatrixXd Hi = model.MakeHessian(i, data_isl)->MakeDenseMatrix();
    std::vector<int> idx;
    for (int c : partition.island_cliques(i)) {
      const int start = model.params().clique_start[c];
      const int size = model.clique_size(c);
      for (int k = 0; k < size; ++k) idx.push_back(start + k);
    }
    ASSERT_EQ(Hi.rows(), ssize(idx));
    H_islands(idx, idx) = Hi;
  }

  EXPECT_NEAR(island_cost_sum, data_full.cost(),
              100 * kEpsilon * std::max(1.0, std::abs(data_full.cost())));
  EXPECT_TRUE(CompareMatrices(data_isl.gradient(), data_full.gradient(),
                              100 * kEpsilon, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(H_islands, H_full, 100 * kEpsilon,
                              MatrixCompareType::relative));

  // Cost-along-line: per-island contributions sum to the full cost-along-line
  // and its derivatives, for an arbitrary search direction.
  const VectorXd w = VectorXd::LinSpaced(nv, 0.1, -0.2);
  IcfSearchDirectionData<double> sdd_full;
  model.CalcSearchDirectionData(data_full, w, &sdd_full);
  IcfSearchDirectionData<double> sdd_i;
  for (double alpha : {-0.3, 0.0, 0.25, 0.8}) {
    double d_full, d2_full;
    const double cost_full =
        model.CalcCostAlongLine(alpha, data_full, sdd_full, &d_full, &d2_full);
    double cost_sum = 0.0, d_sum = 0.0, d2_sum = 0.0;
    for (int i = 0; i < num_islands; ++i) {
      model.CalcSearchDirectionData(data_isl, w, i, &sdd_i);
      double di, d2i;
      cost_sum += model.CalcCostAlongLine(alpha, data_isl, sdd_i, i, &di, &d2i);
      d_sum += di;
      d2_sum += d2i;
    }
    const double tol = 1e-9;
    EXPECT_NEAR(cost_sum, cost_full, tol * std::max(1.0, std::abs(cost_full)));
    EXPECT_NEAR(d_sum, d_full, tol * std::max(1.0, std::abs(d_full)));
    EXPECT_NEAR(d2_sum, d2_full, tol * std::max(1.0, std::abs(d2_full)));
  }
}

/* Many islands (3 separate cliques, no constraints): momentum-only equivalence.
 */
GTEST_TEST(IcfModel, IslandEvalUnconstrained) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();
  ASSERT_EQ(model.partition().num_islands(), 3);
  const VectorXd v = VectorXd::LinSpaced(model.num_velocities(), -10.0, 10.0);
  CheckIslandEquivalence(model, v);
}

/* Two islands with coupler, gain, limit, and patch constraints distributed
across them (no merging weld). */
GTEST_TEST(IcfModel, IslandEvalMultiIsland) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddCouplerConstraint(&model);
  AddGainConstraints(&model);
  AddLimitConstraints(&model);
  AddPatchConstraints(&model);
  model.SetSparsityPattern();
  ASSERT_EQ(model.partition().num_islands(), 2);
  const VectorXd v = VectorXd::LinSpaced(model.num_velocities(), -5.0, 5.0);
  CheckIslandEquivalence(model, v);
}

/* Single island (welds merge everything) with all constraint kinds. The lone
island must reproduce the full problem exactly. */
GTEST_TEST(IcfModel, IslandEvalFullyCoupled) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddCouplerConstraint(&model);
  AddGainConstraints(&model);
  AddLimitConstraints(&model);
  AddPatchConstraints(&model);
  AddWeldConstraints(&model);
  model.SetSparsityPattern();
  ASSERT_EQ(model.partition().num_islands(), 1);
  const VectorXd v = VectorXd::LinSpaced(model.num_velocities(), 0.05, 0.01);
  CheckIslandEquivalence(model, v);
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
