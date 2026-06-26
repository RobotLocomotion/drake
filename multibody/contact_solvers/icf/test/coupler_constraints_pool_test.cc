#include "drake/multibody/contact_solvers/icf/coupler_constraints_pool.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
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

using multibody::internal::SelectRows;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Undo the floating body annotation that conflicts with coupler constraint
// configuration.
void UndoFloatingBodyAnnotation(IcfModel<double>* model) {
  std::unique_ptr<IcfParameters<double>> params = model->ReleaseParameters();
  params->body_is_floating = {0, 0, 0, 0};
  model->ResetParameters(std::move(params));
}

/* Checks that model.CalcData does not incur any heap allocations on a problem
with coupler constraints. */
GTEST_TEST(CouplerConstraintsPool, LimitMallocOnCalcData) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddCouplerConstraint(&model);
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 1);
  EXPECT_EQ(model.num_coupler_constraints(), 1);

  IcfData<double> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.coupler_constraints_data().num_constraints(), 1);

  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);

  // Computing data should not cause any new allocations.
  {
    drake::test::LimitMalloc guard;
    model.CalcData(v, &data);
  }
}

/* Checks that pool.ReduceInto does not incur any heap allocations on a
problem with coupler constraints. */
GTEST_TEST(CouplerConstraintsPool, LimitMallocOnReduceInto) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  AddCouplerConstraint(&model);

  IcfModel<double> reduced_model;
  ReducedMapping mapping;

  // Do a not-smaller reduction to allocate memory in the reduced model.
  MakeModelReducible(&model, /* dofs_to_remove = */ {});
  model.ReduceInto(&reduced_model, &mapping);

  // Given prior allocation of a big enough model, the constraint pool
  // reduction does not allocate.
  {
    drake::test::LimitMalloc guard;
    model.coupler_constraints_pool().ReduceInto(
        mapping, &reduced_model.coupler_constraints_pool());
  }
}

/* Verifies that the coupler constraint produces correct data. */
GTEST_TEST(CouplerConstraintsPool, Data) {
  IcfModel<AutoDiffXd> model;
  MakeUnconstrainedModel(&model);
  model.SetSparsityPattern();
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), 18);
  EXPECT_EQ(model.num_constraints(), 0);

  IcfData<AutoDiffXd> data;
  model.ResizeData(&data);
  EXPECT_EQ(data.num_velocities(), model.num_velocities());

  // At this point there should be no coupler constraints.
  EXPECT_EQ(model.num_coupler_constraints(), 0);

  // Add coupler constraints.
  AddCouplerConstraint(&model);
  EXPECT_EQ(model.num_coupler_constraints(), 1);
  EXPECT_EQ(model.num_constraints(), 1);

  // Resize data to include coupler constraints.
  model.ResizeData(&data);
  EXPECT_EQ(data.coupler_constraints_data().num_constraints(), 1);

  const int nv = model.num_velocities();
  VectorXd v_value = VectorXd::LinSpaced(nv, -10, 10.0);
  VectorX<AutoDiffXd> v(nv);
  math::InitializeAutoDiff(v_value, &v);
  model.CalcData(v, &data);

  const double dt = model.time_step().value();
  const VectorX<AutoDiffXd> q0 = VectorXd::LinSpaced(nv, -1.0, 1.0);
  const double rho = 2.5;
  const double offset = 0.1;
  VectorX<AutoDiffXd> q = q0 + dt * v;
  const auto q_clique = model.clique_segment(1, q);
  const auto v_clique = model.clique_segment(1, v);

  // Compute regularization manually.
  constexpr double kBeta = IcfModel<AutoDiffXd>::kBeta;
  const double m = 2.3;                           // "mass" for clique 1.
  const double w_delassus = (1 + rho * rho) / m;  // Delassus for clique 1.
  const double m_effective = 1.0 / w_delassus;    // Effective mass.
  const double omega_near_rigid =
      2 * M_PI / (kBeta * dt);  // period_nr = kBeta * dt, by definition.
  const double k = m_effective * omega_near_rigid * omega_near_rigid;
  const double tau = kBeta / M_PI * dt;

  const CouplerConstraintsDataPool<AutoDiffXd>& couplers_data =
      data.coupler_constraints_data();
  EXPECT_EQ(couplers_data.num_constraints(), 1);

  const VectorXd cost_gradient = couplers_data.cost().derivatives();

  // Expected impulse.
  const AutoDiffXd g0 = q_clique(1) - rho * q_clique(3) - offset;
  const AutoDiffXd gdot0 = v_clique(1) - rho * v_clique(3);
  const AutoDiffXd gamma0 = -dt * k * (g0 + tau * gdot0);
  VectorXd tau_expected = VectorXd::Zero(nv);
  tau_expected(6 + 1) = gamma0.value();
  tau_expected(6 + 3) = -rho * gamma0.value();
  EXPECT_TRUE(CompareMatrices(-cost_gradient, tau_expected, 10 * kEpsilon,
                              MatrixCompareType::relative));

  const double gamma = couplers_data.gamma(0).value();
  EXPECT_NEAR(gamma, gamma0.value(), std::abs(gamma) * kEpsilon);

  // Verify contributions to Hessian.
  auto hessian = model.MakeHessian(data);
  MatrixXd hessian_value = math::ExtractValue(hessian->MakeDenseMatrix());
  MatrixXd gradient_derivatives = math::ExtractGradient(data.gradient());
  EXPECT_TRUE(CompareMatrices(hessian_value, gradient_derivatives,
                              10 * kEpsilon, MatrixCompareType::relative));

  // Check that CalcCostAlongLine works for coupler constraints.
  // Allocate search direction.
  const VectorX<AutoDiffXd> w = VectorX<AutoDiffXd>::LinSpaced(
      nv, 0.1, -0.2);  // Arbitrary search direction.
  IcfSearchDirectionData<AutoDiffXd> search_data;

  // Set data with constant value of v.
  VectorX<AutoDiffXd> v_constant =
      VectorX<AutoDiffXd>::LinSpaced(nv, -10, 10.0);
  model.CalcData(v_constant, &data);
  model.CalcSearchDirectionData(data, w, &search_data);

  const AutoDiffXd alpha = {
      0.35 /* arbitrary value */,
      VectorXd::Ones(1) /* This is the independent variable */};
  AutoDiffXd dcost, d2cost;
  const AutoDiffXd cost =
      model.CalcCostAlongLine(alpha, data, search_data, &dcost, &d2cost);

  const double scale = std::abs(dcost.value());
  EXPECT_NEAR(dcost.value(), cost.derivatives()[0], scale * kEpsilon);
  EXPECT_NEAR(d2cost.value(), dcost.derivatives()[0], scale * kEpsilon);
}

/* Verifies that reducing the coupler constraint pool produces correct data. */
GTEST_TEST(CouplerConstraintsPool, Reduce) {
  IcfModel<double> model;
  MakeUnconstrainedModel(&model);
  UndoFloatingBodyAnnotation(&model);
  AddCouplerConstraint(&model);

  IcfData<double> data;
  model.ResizeData(&data);
  const int nv = model.num_velocities();
  const VectorXd v = VectorXd::LinSpaced(nv, -10, 10.0);
  // Require all velocities are non-zero.
  ASSERT_TRUE((v.array() != 0).all());
  model.CalcData(v, &data);

  // The R and g_hat stored in the pool are only the time-step independent
  // parts. Calculate the time-step dependent parts here and apply them below.
  const double& dt = model.time_step();
  const double dt_eff = model.effective_time_step();
  constexpr double kBeta = IcfModel<double>::kBeta;
  const double taud = kBeta * dt_eff / M_PI;
  const double R_time_step_factor = (dt_eff * dt_eff) / (dt * (dt + taud));

  auto check_reduced = [&](const std::vector<int>& locked_dofs) {
    SCOPED_TRACE(fmt::format("locked_dofs [{}]", fmt::join(locked_dofs, ", ")));
    IcfModel<double> reduced_model;
    ReducedMapping mapping;
    MakeModelReducible(&model, locked_dofs);
    model.ReduceInto(&reduced_model, &mapping);
    const bool have_i = mapping.velocity_subsequence.participates(7);
    const bool have_j = mapping.velocity_subsequence.participates(9);
    const int expected_constraints = (have_i || have_j);
    ASSERT_EQ(reduced_model.num_coupler_constraints(), expected_constraints);
    if (expected_constraints == 0) {
      return;
    }

    // Check the data transmitted by pool.ReduceInto().
    const auto& full_pool = model.coupler_constraints_pool();
    const auto& reduced_pool = reduced_model.coupler_constraints_pool();
    const int full_clique = full_pool.constraint_to_clique()[0];
    const int reduced_clique = reduced_pool.constraint_to_clique()[0];
    EXPECT_EQ(mapping.clique_subsequence.permuted_index(full_clique),
              reduced_clique);
    const int f_i = full_pool.dofs()[0].first;
    const int f_j = full_pool.dofs()[0].second;
    const int r_i = reduced_pool.dofs()[0].first;
    const int r_j = reduced_pool.dofs()[0].second;
    const auto& clique_mapping = mapping.clique_dof_subsequences[full_clique];
    if (have_i) {
      EXPECT_EQ(clique_mapping.permuted_index(f_i), r_i);
    }
    if (have_j) {
      EXPECT_EQ(clique_mapping.permuted_index(f_j), r_j);
    }
    EXPECT_EQ(full_pool.gear_ratio()[0], reduced_pool.gear_ratio()[0]);
    EXPECT_EQ(full_pool.g_hat()[0], reduced_pool.g_hat()[0]);
    EXPECT_EQ(full_pool.R()[0], reduced_pool.R()[0]);

    // Check the downstream calculations.
    IcfData<double> reduced_data;
    reduced_model.ResizeData(&reduced_data);
    const auto reduced_velocity =
        SelectRows(v, mapping.velocity_subsequence.inverse_permutation());
    reduced_model.CalcData(reduced_velocity, &reduced_data);
    const auto& full_data_pool = data.coupler_constraints_data();
    const auto& reduced_data_pool = reduced_data.coupler_constraints_data();

    // Reconstruct the full regularization.
    const double R = reduced_pool.R()[0] * R_time_step_factor;

    // Check the data pool.
    if (have_i && have_j) {
      // Fully participating reduced constraint data matches the data from the
      // full problem.
      EXPECT_EQ(reduced_data_pool.gamma(0), full_data_pool.gamma(0));
      EXPECT_EQ(reduced_data_pool.cost(), full_data_pool.cost());
    } else {
      // Under the handcrafted test conditions, partially participating reduced
      // constraint data does *not* match the data from the full problem. These
      // checks assume that velocities of locked dofs were non-zero in the full
      // problem.
      EXPECT_NE(reduced_data_pool.gamma(0), full_data_pool.gamma(0));
      EXPECT_NE(reduced_data_pool.cost(), full_data_pool.cost());

      const auto clique_v =
          reduced_model.clique_segment(reduced_clique, reduced_velocity);
      const auto expected_vc =
          (have_i ? clique_v(r_i)
                  : -reduced_pool.gear_ratio()[0] * clique_v(r_j));
      const auto v_hat = reduced_pool.g_hat()[0] / (dt + taud);
      const auto expected_gamma = (v_hat - expected_vc) / R;
      EXPECT_NEAR(reduced_data_pool.gamma(0), expected_gamma, 1e-15);
    }

    // Check gradient contribution.
    VectorXd full_gradient = VectorXd::Zero(model.num_velocities());
    full_pool.AccumulateGradient(data, &full_gradient);
    const auto full_clique_gradient =
        model.clique_segment(full_clique, full_gradient);
    VectorXd reduced_gradient = VectorXd::Zero(reduced_model.num_velocities());
    reduced_pool.AccumulateGradient(reduced_data, &reduced_gradient);
    const auto reduced_clique_gradient =
        reduced_model.clique_segment(reduced_clique, reduced_gradient);
    if (have_i && have_j) {
      // Fully participating reduced constraint data matches the data from the
      // full problem.
      EXPECT_EQ(reduced_clique_gradient(r_i), full_clique_gradient(f_i));
      EXPECT_EQ(reduced_clique_gradient(r_j), full_clique_gradient(f_j));
    } else if (have_i) {
      EXPECT_EQ(reduced_clique_gradient(r_i), -reduced_data_pool.gamma(0));
    } else if (have_j) {
      EXPECT_EQ(reduced_clique_gradient(r_j),
                reduced_pool.gear_ratio()[0] * reduced_data_pool.gamma(0));
    }

    // Check Hessian contribution.
    reduced_model.SetSparsityPattern();
    contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<double>>
        hessian(reduced_model.sparsity_pattern());
    reduced_pool.AccumulateHessian(reduced_data, &hessian);
    if (hessian.HasBlock(reduced_clique, reduced_clique)) {
      const auto& block = hessian.diagonal_block(reduced_clique);
      const double rho = reduced_pool.gear_ratio()[0];
      if (have_i && have_j) {
        EXPECT_EQ(block(r_i, r_j), -rho / R);
        EXPECT_EQ(block(r_i, r_j), block(r_j, r_i));
      }
      if (have_i) {
        EXPECT_EQ(block(r_i, r_i), 1.0 / R);
      }
      if (have_j) {
        EXPECT_EQ(block(r_j, r_j), rho * rho / R);
      }
    }

    // Check cost along line.
    const VectorXd w =
        VectorXd::LinSpaced(nv, 0.1, -0.2);  // Arbitrary search direction.
    double dcost{}, d2cost{};
    full_pool.CalcCostAlongLine(full_data_pool, w, &dcost, &d2cost);

    const auto& reduced_w =
        SelectRows(w, mapping.velocity_subsequence.inverse_permutation());
    double reduced_dcost{}, reduced_d2cost{};
    reduced_pool.CalcCostAlongLine(reduced_data_pool, reduced_w, &reduced_dcost,
                                   &reduced_d2cost);
    if (have_i && have_j) {
      EXPECT_EQ(reduced_dcost, dcost);
      EXPECT_EQ(reduced_d2cost, d2cost);
    } else {
      // Partially participating reduced constraint data does *not* match the
      // data from the full problem. These checks assume that velocities of
      // locked dofs were non-zero in the full problem.
      EXPECT_NE(reduced_dcost, dcost);
      EXPECT_NE(reduced_d2cost, d2cost);

      const auto clique_w =
          reduced_model.clique_segment(reduced_clique, reduced_w);
      const auto expected_vw =
          (have_i ? clique_w(r_i)
                  : -reduced_pool.gear_ratio()[0] * clique_w(r_j));
      const auto expected_dcost = -reduced_data_pool.gamma(0) * expected_vw;
      const auto expected_d2cost = expected_vw * expected_vw / R;
      EXPECT_NEAR(reduced_dcost, expected_dcost, 1e-12);
      EXPECT_NEAR(reduced_d2cost, expected_d2cost, 1e-15);
    }
  };

  // Reduce by none; essentially, just copy.
  const std::vector<int> none_locked;
  check_reduced(none_locked);

  // Lock some arbitrary dofs.
  const std::vector<int> arbitrary_locked = {0, 17};
  check_reduced(arbitrary_locked);

  // Lock one constraint dof.
  const std::vector<int> constraint_dof_i_locked = {7};
  check_reduced(constraint_dof_i_locked);

  // Lock other constraint dof.
  const std::vector<int> constraint_dof_j_locked = {9};
  check_reduced(constraint_dof_j_locked);

  // Lock both constraint dofs.
  const std::vector<int> constraint_dofs_both_locked = {7, 9};
  check_reduced(constraint_dofs_both_locked);

  // Lock other dofs in constraint clique.
  const std::vector<int> other_dofs_locked = {6, 8};
  check_reduced(other_dofs_locked);

  // Lock an entire arbitrary clique.
  const std::vector<int> clique0_locked = {0, 1, 2, 3, 4, 5};
  check_reduced(clique0_locked);

  // Lock the entire constrained clique.
  const std::vector<int> clique1_locked = {6, 7, 8, 9, 10, 11};
  check_reduced(clique1_locked);

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
