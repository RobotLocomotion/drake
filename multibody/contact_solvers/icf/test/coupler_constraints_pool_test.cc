#include "drake/multibody/contact_solvers/icf/coupler_constraints_pool.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
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

/* Checks that model.CalcData does not incur any heap allocations on a problem
with coupler constraints. */
GTEST_TEST(IcfModel, LimitMallocOnCouplerConstrainedCalcData) {
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

/* Verifies that the coupler constraint produces correct data. */
GTEST_TEST(IcfModel, CouplerConstraint) {
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
  const double beta =
      0.1;  // Keep in sync with hard-coded value in the implementation.
  const double m = 2.3;                           // "mass" for clique 1.
  const double w_delassus = (1 + rho * rho) / m;  // Delassus for clique 1.
  const double m_effective = 1.0 / w_delassus;    // Effective mass.
  const double omega_near_rigid =
      2 * M_PI / (beta * dt);  // period_nr = beta * dt, by definition.
  const double k = m_effective * omega_near_rigid * omega_near_rigid;
  const double tau = beta / M_PI * dt;

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

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
