#include <gtest/gtest.h>

#include "drake/examples/pendulum/pendulum_plant.h"

namespace drake {
namespace systems {
namespace {

using Eigen::MatrixXd;
using examples::pendulum::PendulumPlant;
using examples::pendulum::PendulumState;
using examples::pendulum::PendulumStateIndices;

// N.B. The test code immediately below must be kept in sync with the identical
// code in ../system_scalar_conversion_doxygen.h.
GTEST_TEST(SystemScalarConversionDoxygen, PendulumPlantAutodiff) {
  // Establish the plant and its initial conditions:
  //   tau = 0, theta = 0.1, thetadot = 0.2.
  auto plant = std::make_unique<PendulumPlant<double>>();
  auto context = plant->CreateDefaultContext();
  plant->get_input_port(0).FixValue(context.get(), 0.0);  // tau
  auto* state = dynamic_cast<PendulumState<double>*>(
      &context->get_mutable_continuous_state_vector());
  state->set_theta(0.1);
  state->set_thetadot(0.2);
  double energy = plant->CalcTotalEnergy(*context);
  ASSERT_NEAR(energy, -4.875, 0.001);

  // Convert the plant and its context to use AutoDiff.
  auto autodiff_plant = System<double>::ToAutoDiffXd(*plant);
  auto autodiff_context = autodiff_plant->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(*context);
  autodiff_plant->FixInputPortsFrom(*plant, *context, autodiff_context.get());

  // Differentiate with respect to theta by setting dtheta/dtheta = 1.0.
  constexpr int kNumDerivatives = 1;
  auto& xc = autodiff_context->get_mutable_continuous_state_vector();
  xc[PendulumStateIndices::kTheta].derivatives() =
      MatrixXd::Identity(kNumDerivatives, kNumDerivatives).col(0);

  // TODO(#6944) This is a hack to work around AutoDiffXd being broken.
  // (This stanza is excluded from the Doxygen twin of this unit test.)
  auto& params = autodiff_context->get_mutable_numeric_parameter(0);
  for (int i = 0; i < params.size(); ++i) {
    params[i].derivatives() = Vector1d::Zero(1);
  }

  // Compute denergy/dtheta around its initial conditions.
  AutoDiffXd autodiff_energy =
      autodiff_plant->CalcTotalEnergy(*autodiff_context);
  ASSERT_NEAR(autodiff_energy.value(), -4.875, 0.001);
  ASSERT_EQ(autodiff_energy.derivatives().size(), 1);
  ASSERT_NEAR(autodiff_energy.derivatives()[0], 0.490, 0.001);
}

}  // namespace
}  // namespace systems
}  // namespace drake
