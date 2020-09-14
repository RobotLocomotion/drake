#include "drake/examples/acrobot/acrobot_plant.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

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
  EXPECT_LT(residual.lpNorm<Eigen::Infinity>(), 100*kEpsilon);
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

