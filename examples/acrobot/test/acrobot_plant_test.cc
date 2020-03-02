#include "drake/examples/acrobot/acrobot_plant.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

GTEST_TEST(AcrobotPlantTest, MassMatrixFormTest) {
  AcrobotPlant<double> plant;
  auto context = plant.CreateDefaultContext();
  context->FixInputPort(0, Vector1d(-.32));
  context->SetContinuousState(Eigen::Vector4d(.1, .2, .3, .4));

  Eigen::MatrixXd mass_matrix;
  Eigen::VectorXd right_hand_side;

  const systems::ContinuousState<double>& derivatives =
      plant.EvalTimeDerivatives(*context);
  plant.CalcTimeDerivativesMassMatrixForm(*context, &mass_matrix,
                                          &right_hand_side);
  EXPECT_TRUE(CompareMatrices(mass_matrix * derivatives.CopyToVector(),
                              right_hand_side, 1e-14));
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
