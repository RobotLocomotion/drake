#include "drake/examples/acrobot/acrobot_plant.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

GTEST_TEST(AcrobotPlantTest, ImplicitTimeDerivatives) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();

  AcrobotPlant<double> plant;
  auto context = plant.CreateDefaultContext();
  plant.get_input_port(0).FixValue(context.get(), -.32);
  context->SetContinuousState(Eigen::Vector4d(.1, .2, .3, .4));

  Eigen::VectorXd residual = plant.AllocateImplicitTimeDerivativesResidual();

  // Try perfect proposed derivatives -- should get near-zero residual.
  const systems::ContinuousState<double>& proposed_derivatives =
      plant.EvalTimeDerivatives(*context);
  plant.CalcImplicitTimeDerivativesResidual(*context, proposed_derivatives,
                                            &residual);

  // Expected accuracy depends on the mass matrix condition number (about 41
  // here) and the solve accuracy. We're limited to using Eigen's M.inverse()
  // here to permit this to be done symbolically, which is not the most
  // accurate method. 100ε (~2e-14) should be achievable.
  EXPECT_LT(residual.lpNorm<Eigen::Infinity>(), 100 * kEpsilon);
}

// Ensure that DoCalcTimeDerivatives succeeds even if the input port is
// disconnected.
GTEST_TEST(AcrobotPlantTest, NoInput) {
  const AcrobotPlant<double> plant;
  auto context = plant.CreateDefaultContext();

  DRAKE_EXPECT_NO_THROW(plant.EvalTimeDerivatives(*context));
}

GTEST_TEST(AcrobotPlantTest, SetMitAcrobotParameters) {
  const AcrobotPlant<double> plant;
  auto context = plant.CreateDefaultContext();
  auto& parameters = plant.get_mutable_parameters(context.get());
  const double m1_old = parameters.m1();
  plant.SetMitAcrobotParameters(&parameters);
  const double m1_mit = parameters.m1();
  EXPECT_NE(m1_old, m1_mit);
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
