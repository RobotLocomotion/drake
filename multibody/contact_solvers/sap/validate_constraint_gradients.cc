#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

#include <limits>
#include <memory>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

void ValidateConstraintGradients(const SapConstraint<AutoDiffXd>& c,
                                 const AbstractValue& data) {
  const int nk = c.num_constraint_equations();

  // Verify that minus the gradient of the cost is the impulse.
  const AutoDiffXd cost_ad = c.CalcCost(data);
  VectorX<AutoDiffXd> gamma_ad(nk);
  c.CalcImpulse(data, &gamma_ad);
  const VectorXd gamma = math::ExtractValue(gamma_ad);
  const VectorXd minus_cost_gradient = (cost_ad.derivatives().size() == 0)
                                           ? VectorXd::Zero(nk)
                                           : VectorXd(-cost_ad.derivatives());
  EXPECT_TRUE(CompareMatrices(gamma, minus_cost_gradient,
                              20 * std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));

  // Verify that minus the gradient of the impulse is the Hessian.
  MatrixX<AutoDiffXd> G_ad(nk, nk);
  c.CalcCostHessian(data, &G_ad);
  const MatrixXd G = math::ExtractValue(G_ad);
  const MatrixXd minus_gamma_ad_gradient = -math::ExtractGradient(gamma_ad, nk);
  EXPECT_TRUE(CompareMatrices(G, minus_gamma_ad_gradient,
                              32 * std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

void ValidateConstraintGradients(const SapConstraint<AutoDiffXd>& c,
                                 const VectorXd& vc) {
  VectorX<AutoDiffXd> vc_ad = drake::math::InitializeAutoDiff(vc);

  const int nk = c.num_constraint_equations();
  // Arbitrary time step and Delassus operator estimation.
  const AutoDiffXd time_step = 0.01;
  VectorX<AutoDiffXd> delassus_estimation =
      VectorX<AutoDiffXd>::Constant(nk, 1.0);

  // Make data and update for the input value of vc.
  std::unique_ptr<AbstractValue> data =
      c.MakeData(time_step, delassus_estimation);
  c.CalcData(vc_ad, data.get());

  ValidateConstraintGradients(c, *data);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
