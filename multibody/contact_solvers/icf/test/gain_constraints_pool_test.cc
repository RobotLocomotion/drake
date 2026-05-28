#include "drake/multibody/contact_solvers/icf/gain_constraints_pool.h"

#include <limits>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/test_utilities/icf_model_test_helpers.h"

using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

/* Checks that model.CalcData does not incur any heap allocations on a problem
with gain constraints. */
GTEST_TEST(GainConstraintsPool, LimitMallocOnCalcData) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddGainConstraints(&model);
  EXPECT_EQ(model.num_constraints(), 2);
  EXPECT_EQ(model.num_gain_constraints(), 2);

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.gain_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10.0, 10.0);

  // Computing data should not cause any new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

/* Checks that pool.ReduceInto does not incur any heap allocations on a
problem with gain constraints. */
GTEST_TEST(GainConstraintsPool, LimitMallocOnReduceInto) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddGainConstraints(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  // Do a not-smaller reduction to allocate memory in the reduced model.
  MakeModelReducible(&model, /* dofs_to_remove = */ {});
  model.ReduceInto(&reduced_model, &mapping);

  // Given prior allocation of a big enough model, the couple constraint pool
  // reduction does not allocate.
  {
    // TODO(#23912): consider removing these allocations.
    drake::test::LimitMalloc guard({16});
    model.gain_constraints_pool().ReduceInto(
        mapping, &reduced_model.gain_constraints_pool());
  }
}

/* Verifies that gain constraints produce correct data. */
GTEST_TEST(GainConstraintsPool, Data) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 0);

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.gain_constraints_data().num_constraints(), 0);

  // At this point there should be no gain constraints.
  EXPECT_EQ(model.num_gain_constraints(), 0);

  // Add gain constraints.
  std::vector<VectorX<AutoDiffXd>> gains = AddGainConstraints(&model);
  DRAKE_DEMAND(ssize(gains) == 6);
  const VectorX<AutoDiffXd>& K0 = gains[0];
  const VectorX<AutoDiffXd>& u0 = gains[1];
  const VectorX<AutoDiffXd>& e0 = gains[2];
  const VectorX<AutoDiffXd>& K2 = gains[3];
  const VectorX<AutoDiffXd>& u2 = gains[4];
  const VectorX<AutoDiffXd>& e2 = gains[5];

  EXPECT_EQ(model.num_gain_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 2);

  // Resize data to include gain constraints data.
  model.ResizeData(&data);
  EXPECT_EQ(data.gain_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const GainConstraintsDataPool<AutoDiffXd>& gains_data =
      data.gain_constraints_data();
  EXPECT_EQ(gains_data.num_constraints(), 2);

  const VectorXd K0_value = math::ExtractValue(K0);
  const VectorXd u0_value = math::ExtractValue(u0);
  const VectorXd e0_value = math::ExtractValue(e0);
  const VectorXd v0_value = v_value.segment<6>(0);  // Clique 0's values.
  VectorXd tau0_expected = -K0_value.cwiseProduct(v0_value) + u0_value;

  const VectorXd K2_value = math::ExtractValue(K2);
  const VectorXd u2_value = math::ExtractValue(u2);
  const VectorXd e2_value = math::ExtractValue(e2);
  const VectorXd v2_value = v_value.segment<6>(12);  // Clique 2's values.
  VectorXd tau2_expected = -K2_value.cwiseProduct(v2_value) + u2_value;

  // For this test we verify some values are below, within and above effort
  // limits (+/- 0.9).
  EXPECT_GT(tau0_expected(0), 0.9);
  EXPECT_GT(tau0_expected(1), 0.9);
  EXPECT_GT(tau0_expected(2), 0.9);
  EXPECT_GT(tau0_expected(3), 0.9);
  EXPECT_GT(tau0_expected(4), -0.9);
  EXPECT_LT(tau0_expected(4), 0.9);
  EXPECT_LT(tau0_expected(5), -0.9);

  // Apply the limits.
  tau0_expected = tau0_expected.cwiseMax(-e0_value).cwiseMin(e0_value);
  tau2_expected = tau2_expected.cwiseMax(-e2_value).cwiseMin(e2_value);
  const VectorXd gamma0_expected = tau0_expected * model.time_step().value();
  const VectorXd gamma2_expected = tau2_expected * model.time_step().value();

  const VectorX<AutoDiffXd> gamma0 = gains_data.gamma(0);
  const VectorX<AutoDiffXd> gamma2 = gains_data.gamma(1 /* constraint index */);
  EXPECT_EQ(gamma0.size(), model.clique_size(0));
  EXPECT_EQ(gamma2.size(), model.clique_size(2));
  const VectorXd gamma0_value = math::ExtractValue(gamma0);
  const VectorXd gamma2_value = math::ExtractValue(gamma2);
  EXPECT_TRUE(CompareMatrices(gamma0_value, gamma0_expected, kEpsilon,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(gamma2_value, gamma2_expected, kEpsilon,
                              MatrixCompareType::relative));

  // Gradient of the cost.
  const VectorXd cost_gradient = gains_data.cost().derivatives();
  const VectorXd minus_cost0_gradient = -cost_gradient.segment<6>(0);
  const VectorXd minus_cost2_gradient =
      -cost_gradient.segment<6>(12 /* first velocity index */);
  EXPECT_TRUE(CompareMatrices(gamma0_value, minus_cost0_gradient, kEpsilon,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(gamma2_value, minus_cost2_gradient, kEpsilon,
                              MatrixCompareType::relative));

  // Verify accumulated total cost and gradients.
  const VectorXd total_cost_derivatives = data.cost().derivatives();
  const VectorXd total_gradient_value = math::ExtractValue(data.gradient());

  EXPECT_TRUE(CompareMatrices(total_gradient_value, total_cost_derivatives,
                              2 * kEpsilon, MatrixCompareType::relative));
}

/* Verifies that reducing the gain constraint pool produces correct data. */
GTEST_TEST(GainConstraintsPool, Reduce) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddGainConstraints(&model);

  IcfData<double> data;
  model.ResizeData(&data);
  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model.CalcData(v, &data);
  const auto& full_pool = model.gain_constraints_pool();

  auto check_reduced = [&](const std::vector<int>& locked_dofs) {
    SCOPED_TRACE(fmt::format("locked_dofs [{}]", fmt::join(locked_dofs, ", ")));
    IcfModel<double> reduced_model;
    ReducedMapping mapping;
    MakeModelReducible(&model, locked_dofs);
    model.ReduceInto(&reduced_model, &mapping);
    const auto& reduced_pool = reduced_model.gain_constraints_pool();
    int reduced_constraint_cursor{0};
    for (int k = 0; k < full_pool.num_constraints(); ++k) {
      SCOPED_TRACE(fmt::format("full constraint {} vs. reduced constraint {}",
                               k, reduced_constraint_cursor));
      const int full_clique = full_pool.clique()[k];
      if (!mapping.clique_subsequence.participates(full_clique)) {
        continue;
      }

      // Check the data transmitted by pool.ReduceInto().
      const auto& dof_subsequence =
          mapping.clique_dof_subsequences[full_clique];
      const auto& indices = dof_subsequence.inverse_permutation();
      const int reduced_clique =
          reduced_pool.clique()[reduced_constraint_cursor];
      EXPECT_EQ(mapping.clique_subsequence.permuted_index(full_clique),
                reduced_clique);
      EXPECT_EQ(dof_subsequence.permuted_domain_size(),
                reduced_pool.constraint_size()[reduced_constraint_cursor]);
      EXPECT_TRUE(CompareMatrices(full_pool.K()[k](indices),
                                  reduced_pool.K()[reduced_constraint_cursor]));
      EXPECT_TRUE(CompareMatrices(full_pool.b()[k](indices),
                                  reduced_pool.b()[reduced_constraint_cursor]));
      EXPECT_TRUE(
          CompareMatrices(full_pool.le()[k](indices),
                          reduced_pool.le()[reduced_constraint_cursor]));
      EXPECT_TRUE(
          CompareMatrices(full_pool.ue()[k](indices),
                          reduced_pool.ue()[reduced_constraint_cursor]));

      ++reduced_constraint_cursor;
    }
    EXPECT_EQ(reduced_model.num_gain_constraints(), reduced_constraint_cursor);
  };

  // Reduce by none; essentially, just copy.
  const std::vector<int> none_locked;
  check_reduced(none_locked);

  // Lock some arbitrary dofs.
  const std::vector<int> arbitrary_locked = {0, 17};
  check_reduced(arbitrary_locked);

  // Lock a constrained clique.
  const std::vector<int> clique0_locked = {0, 1, 2, 3, 4, 5};
  check_reduced(clique0_locked);

  // Lock an unconstrained clique.
  const std::vector<int> clique1_locked = {6, 7, 8, 9, 10, 11};
  check_reduced(clique1_locked);

  // Lock other constrained clique.
  const std::vector<int> clique2_locked = {12, 13, 14, 15, 16, 17};
  check_reduced(clique2_locked);

  // Lock everything.
  std::vector<int> all_locked(model.num_velocities());
  std::iota(all_locked.begin(), all_locked.end(), 0);
  check_reduced(all_locked);
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
