#include "drake/multibody/contact_solvers/icf/limit_constraints_pool.h"

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

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

/* Checks that model.CalcData does not incur any heap allocations on a problem
with limit constraints. */
GTEST_TEST(LimitConstraintsPool, LimitMallocOnCalcData) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddLimitConstraints(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 2);
  EXPECT_EQ(model.num_limit_constraints(), 2);

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.limit_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10.0, 10.0);

  // Computing data should not cause any new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

/* Checks that pool.ReduceInto does not incur any heap allocations on a
problem with limit constraints. */
GTEST_TEST(LimitConstraintsPool, LimitMallocOnReduceInto) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddLimitConstraints(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  // Do a not-smaller reduction to allocate memory in the reduced model.
  MakeModelReducible(&model, {});
  model.ReduceInto(&reduced_model, &mapping);

  // Given prior allocation of a big enough model, the constraint pool
  // reduction does not allocate.
  {
    // TODO(#23912): consider reducing these allocations.
    drake::test::LimitMalloc guard({24});
    model.limit_constraints_pool().ReduceInto(
        mapping, &reduced_model.limit_constraints_pool());
  }
}

/* Verifies that limit constraints produce correct data. */
GTEST_TEST(LimitConstraintsPool, Data) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 0);

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());
  EXPECT_EQ(data.limit_constraints_data().num_constraints(), 0);

  // At this point there should be no limit constraints.
  EXPECT_EQ(model.num_limit_constraints(), 0);
  EXPECT_EQ(model.num_constraints(), 0);

  // Add a few limit constraints to the model.
  AddLimitConstraints(&model);
  EXPECT_EQ(model.num_limit_constraints(), 2);
  EXPECT_EQ(model.num_constraints(), 2);

  // Resize data to include limit constraints data.
  model.ResizeData(&data);
  EXPECT_EQ(data.limit_constraints_data().num_constraints(), 2);

  const int nv = model.num_velocities();
  const VectorX<AutoDiffXd> q0 = VectorXd::LinSpaced(nv, -1.0, 1.0);
  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const AutoDiffXd dt = model.time_step();
  VectorX<AutoDiffXd> q = q0 + dt * v;

  const LimitConstraintsDataPool<AutoDiffXd>& limits_data =
      data.limit_constraints_data();
  EXPECT_EQ(limits_data.num_constraints(), 2);

  const VectorXd cost_gradient = limits_data.cost().derivatives();

  // Impulses on constraint 0, clique 0.
  const VectorX<AutoDiffXd> gamma_lower0 = limits_data.gamma_lower(0);
  const VectorX<AutoDiffXd> gamma_upper0 = limits_data.gamma_upper(0);
  // tau = Jᵀ⋅γ, and J = [1; -1] for limit constraints.
  const VectorX<AutoDiffXd> tau0 = gamma_lower0 - gamma_upper0;

  // Impulses on constraint 1, clique 2.
  const VectorX<AutoDiffXd> gamma_lower1 = limits_data.gamma_lower(1);
  const VectorX<AutoDiffXd> gamma_upper1 = limits_data.gamma_upper(1);
  // tau = Jᵀ⋅γ, and J = [1; -1] for limit constraints.
  const VectorX<AutoDiffXd> tau1 = gamma_lower1 - gamma_upper1;

  // Assemble all clique contributions into tau by hand.
  VectorX<AutoDiffXd> tau = VectorX<AutoDiffXd>::Zero(nv);
  tau.segment<6>(0) = tau0;
  tau.segment<6>(12) = tau1;

  // Verify that the impulses are non-zero, e.g., at least one limit is
  // active.
  EXPECT_TRUE(tau.norm() > 1e-6);

  // Verify that the cost gradient matches the expected impulse.
  const VectorXd tau_value = math::ExtractValue(tau);
  EXPECT_TRUE(CompareMatrices(cost_gradient, -tau_value, kEpsilon,
                              MatrixCompareType::relative));

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.gradient());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives,
                              10 * kEpsilon, MatrixCompareType::relative));
}

/* Verifies that reducing the limit constraint pool produces correct data. */
GTEST_TEST(LimitConstraintsPool, Reduce) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddLimitConstraints(&model);

  IcfData<double> data;
  model.ResizeData(&data);
  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  model.CalcData(v, &data);
  const auto& full_pool = model.limit_constraints_pool();

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  auto check_reduced = [&](const std::vector<int>& locked_dofs) {
    SCOPED_TRACE(fmt::format("locked_dofs [{}]", fmt::join(locked_dofs, ", ")));
    MakeModelReducible(&model, locked_dofs);
    model.ReduceInto(&reduced_model, &mapping);
    const auto& reduced_pool = reduced_model.limit_constraints_pool();
    int reduced_constraint_cursor{0};
    for (int k = 0; k < full_pool.num_constraints(); ++k) {
      SCOPED_TRACE(fmt::format("full constraint {} vs. reduced constraint {}",
                               k, reduced_constraint_cursor));
      int full_clique = full_pool.clique()[k];
      if (!mapping.clique_subsequence.participates(full_clique)) {
        continue;
      }
      const auto& dof_subsequence =
          mapping.clique_dof_subsequences[full_clique];
      int full_dof = full_pool.dof()[k];
      if (!dof_subsequence.participates(full_dof)) {
        continue;
      }

      // Check the data transmitted by pool.ReduceInto().
      const auto& indices = dof_subsequence.inverse_permutation();
      int reduced_clique = reduced_pool.clique()[reduced_constraint_cursor];
      EXPECT_EQ(reduced_clique,
                mapping.clique_subsequence.permuted_index(full_clique));
      int reduced_dof = reduced_pool.dof()[reduced_constraint_cursor];
      EXPECT_EQ(reduced_dof, dof_subsequence.permuted_index(full_dof));
      EXPECT_EQ(reduced_pool.constraint_size()[reduced_constraint_cursor],
                dof_subsequence.permuted_domain_size());
      EXPECT_TRUE(CompareMatrices(reduced_pool.ql()[reduced_constraint_cursor],
                                  full_pool.ql()[k](indices)));
      EXPECT_TRUE(CompareMatrices(reduced_pool.qu()[reduced_constraint_cursor],
                                  full_pool.qu()[k](indices)));
      EXPECT_TRUE(CompareMatrices(reduced_pool.q0()[reduced_constraint_cursor],
                                  full_pool.q0()[k](indices)));
      EXPECT_TRUE(CompareMatrices(reduced_pool.gl0()[reduced_constraint_cursor],
                                  full_pool.gl0()[k](indices)));
      EXPECT_TRUE(CompareMatrices(reduced_pool.gu0()[reduced_constraint_cursor],
                                  full_pool.gu0()[k](indices)));
      EXPECT_TRUE(
          CompareMatrices(reduced_pool.R_fragment()[reduced_constraint_cursor],
                          full_pool.R_fragment()[k](indices)));

      ++reduced_constraint_cursor;
    }
    EXPECT_EQ(reduced_model.num_limit_constraints(), reduced_constraint_cursor);
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
