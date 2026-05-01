#include "drake/multibody/contact_solvers/icf/icf_model.h"

#include <limits>
#include <memory>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_search_direction_data.h"
#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"
#include "drake/multibody/plant/slicing_and_indexing.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

using multibody::internal::ExpandRows;
using multibody::internal::SelectRows;
using multibody::internal::SelectRowsCols;

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

  IcfModel<double> reduced_model;
  ReducedMapping mapping;
  // Do a non-reducing reduce to allocate storage in the reduced model.
  model.ReduceInto(&reduced_model, &mapping);

  // Reducing into a same size model should cause no new allocations.
  {
    // XXX measured 10:
    // 4 in ReduceInto itself (selects on Eigen params).
    // 6 in SetSparsityPattern.
    drake::test::LimitMalloc guard({10});
    model.ReduceInto(&reduced_model, &mapping);
  }

  // Reducing into a smaller model should have minimal allocations.
  {
    MakeModelReducible(&model, {2, 16});

    // XXX measured 13:
    // 4 in ReduceInto itself (selects on Eigen params).
    // 3 in ResetParameters.
    // 6 in SetSparsityPattern.
    drake::test::LimitMalloc guard({13});
    model.ReduceInto(&reduced_model, &mapping);
  }
}

GTEST_TEST(IcfModel, ReducedMapping) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  auto check_permutations_are_sorted = [](const ReducedMapping& m,
                                          std::string_view message) {
    SCOPED_TRACE(message);
    EXPECT_TRUE(
        std::ranges::is_sorted(m.velocity_permutation.inverse_permutation()));
    EXPECT_TRUE(
        std::ranges::is_sorted(m.clique_permutation.inverse_permutation()));
    for (const auto& clique_dofs : m.clique_dof_permutations) {
      EXPECT_TRUE(std::ranges::is_sorted(clique_dofs.inverse_permutation()));
    }
  };

  // Lock some arbitrary dofs.
  const std::vector<int> arbitrary_locked = {0, 17};
  MakeModelReducible(&model, arbitrary_locked);
  model.ReduceInto(&reduced_model, &mapping);
  check_permutations_are_sorted(mapping, "arbitrary dofs");
  EXPECT_EQ(mapping.velocity_permutation.permuted_domain_size(), 16);
  for (auto dof : arbitrary_locked) {
    EXPECT_FALSE(mapping.velocity_permutation.participates(dof));
  }
  EXPECT_EQ(mapping.clique_permutation.permuted_domain_size(), 3);
  EXPECT_EQ(mapping.clique_dof_permutations[0].permuted_domain_size(), 5);
  EXPECT_FALSE(mapping.clique_dof_permutations[0].participates(0));
  EXPECT_EQ(mapping.clique_dof_permutations[1].permuted_domain_size(), 6);
  EXPECT_EQ(mapping.clique_dof_permutations[2].permuted_domain_size(), 5);
  EXPECT_FALSE(mapping.clique_dof_permutations[2].participates(5));

  // Lock an entire clique.
  const std::vector<int> clique0_locked = {0, 1, 2, 3, 4, 5};
  MakeModelReducible(&model, clique0_locked);
  model.ReduceInto(&reduced_model, &mapping);
  check_permutations_are_sorted(mapping, "clique0 locked");
  EXPECT_EQ(mapping.velocity_permutation.permuted_domain_size(), 12);
  for (auto dof : clique0_locked) {
    EXPECT_FALSE(mapping.velocity_permutation.participates(dof));
  }
  EXPECT_EQ(mapping.clique_permutation.permuted_domain_size(), 2);
  EXPECT_FALSE(mapping.clique_permutation.participates(0));
  EXPECT_EQ(mapping.clique_dof_permutations[0].permuted_domain_size(), 0);
  EXPECT_EQ(mapping.clique_dof_permutations[1].permuted_domain_size(), 6);
  EXPECT_EQ(mapping.clique_dof_permutations[2].permuted_domain_size(), 6);

  // Lock everything.
  std::vector<int> all_locked(model.num_velocities());
  std::iota(all_locked.begin(), all_locked.end(), 0);
  MakeModelReducible(&model, all_locked);
  model.ReduceInto(&reduced_model, &mapping);
  check_permutations_are_sorted(mapping, "all locked");
  EXPECT_EQ(mapping.velocity_permutation.permuted_domain_size(), 0);
  for (auto dof : all_locked) {
    EXPECT_FALSE(mapping.velocity_permutation.participates(dof));
  }
  EXPECT_EQ(mapping.clique_permutation.permuted_domain_size(), 0);
  EXPECT_FALSE(mapping.clique_permutation.participates(0));
  EXPECT_EQ(mapping.clique_dof_permutations[0].permuted_domain_size(), 0);
  EXPECT_EQ(mapping.clique_dof_permutations[1].permuted_domain_size(), 0);
  EXPECT_EQ(mapping.clique_dof_permutations[2].permuted_domain_size(), 0);
}

GTEST_TEST(IcfModel, ReducedDataAndHessian) {
  IcfModel<double> model;
  IcfData<double> data;
  std::unique_ptr<
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixXd>>
      hessian;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();
  model.ResizeData(&data);
  model.CalcData(VectorX<double>::Zero(model.num_velocities()), &data);
  hessian = model.MakeHessian(data);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;
  VectorX<double> reduced_v;
  IcfData<double> reduced_data;
  std::unique_ptr<
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixXd>>
      reduced_hessian;

  // The Hessian made by the reduced model should be the same as the
  // unlocked-dofs slice of the Hessian made by the full model.
  auto check_reduced_hessian = [&](const std::vector<int>& locked_dofs) {
    MakeModelReducible(&model, locked_dofs);
    model.ReduceInto(&reduced_model, &mapping);
    reduced_v = VectorX<double>::Zero(reduced_model.num_velocities());
    reduced_model.ResizeData(&reduced_data);
    reduced_model.CalcData(reduced_v, &reduced_data);
    reduced_hessian = reduced_model.MakeHessian(reduced_data);
    auto sliced_hessian =
        SelectRowsCols(hessian->MakeDenseMatrix(),
                       mapping.velocity_permutation.inverse_permutation());
    EXPECT_TRUE(
        CompareMatrices(sliced_hessian, reduced_hessian->MakeDenseMatrix()));
  };

  // Reduce by none; essentially, just copy.
  const std::vector<int> none_locked;
  check_reduced_hessian(none_locked);

  // Lock some arbitrary dofs.
  const std::vector<int> arbitrary_locked = {0, 17};
  check_reduced_hessian(arbitrary_locked);

  // Lock an entire clique.
  const std::vector<int> clique0_locked = {0, 1, 2, 3, 4, 5};
  check_reduced_hessian(clique0_locked);

  // Lock an entire clique.
  const std::vector<int> clique1_locked = {6, 7, 8, 9, 10, 11};
  check_reduced_hessian(clique1_locked);

  // Lock everything.
  std::vector<int> all_locked(model.num_velocities());
  std::iota(all_locked.begin(), all_locked.end(), 0);
  check_reduced_hessian(all_locked);
}

GTEST_TEST(IcfModel, ReducedV_WB0) {
  IcfModel<double> model;
  IcfData<double> data;
  MakeUnconstrainedModel(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  // The V_WB0s made by the reduced model are one of:
  // * 0, if all dofs are locked,
  // * equal to the full model, if the body is floating or anchored,
  // * equal to the full model's Jacobian, multiplied by a velocity with locked
  // * dofs set to 0.
  auto check_reduced_V_WB0 = [&](const std::vector<int>& locked_dofs) {
    SCOPED_TRACE(fmt::format("locked_dofs {}", fmt::join(locked_dofs, ", ")));
    MakeModelReducible(&model, locked_dofs);
    model.ReduceInto(&reduced_model, &mapping);
    for (int b = 0; b < model.num_bodies(); ++b) {
      SCOPED_TRACE(fmt::format("body {}", b));
      Vector6<double> V_WB0 = model.V_WB0(b);
      Vector6<double> reduced_V_WB0 = reduced_model.V_WB0(b);
      if (mapping.velocity_permutation.permuted_domain_size() == 0) {
        EXPECT_TRUE(CompareMatrices(Vector6<double>::Zero(), reduced_V_WB0));
      } else if (model.is_floating(b) || model.is_anchored(b)) {
        EXPECT_TRUE(CompareMatrices(V_WB0, reduced_V_WB0));
      } else {
        auto unlocked_dofs = mapping.velocity_permutation.inverse_permutation();
        // This velocity has 0 for locked dofs.
        auto re_expanded_velocity =
            ExpandRows(SelectRows(model.v0(), unlocked_dofs),
                       model.num_velocities(), unlocked_dofs);
        int clique = model.params().body_to_clique[b];
        // This velocity has 0 for locked dofs.
        auto clique_velocity =
            model.clique_segment(clique, re_expanded_velocity);
        auto expected_V_WB0 = model.params().J_WB[b] * clique_velocity;
        EXPECT_TRUE(CompareMatrices(expected_V_WB0, reduced_V_WB0));
      }
    }
  };

  // Reduce by none; essentially, just copy.
  const std::vector<int> none_locked;
  check_reduced_V_WB0(none_locked);

  // Lock some arbitrary dofs.
  const std::vector<int> arbitrary_locked = {0, 17};
  check_reduced_V_WB0(arbitrary_locked);

  // Lock an entire clique.
  const std::vector<int> clique0_locked = {0, 1, 2, 3, 4, 5};
  check_reduced_V_WB0(clique0_locked);

  // Lock everything.
  std::vector<int> all_locked(model.num_velocities());
  std::iota(all_locked.begin(), all_locked.end(), 0);
  check_reduced_V_WB0(all_locked);
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

/* Checks that we get the same result for the same problem, regardless of how
we break out the cliques. */
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

/* Checks that updating the time step produces the same result as creating a
new model from scratch. */
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

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
