#include "drake/multibody/contact_solvers/icf/gain_constraints_pool.h"

#include <limits>
#include <vector>

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

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
