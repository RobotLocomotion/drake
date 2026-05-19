#include "drake/multibody/contact_solvers/icf/limit_constraints_pool.h"

#include <limits>

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

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
