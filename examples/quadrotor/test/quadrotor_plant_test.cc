#include "drake/examples/quadrotor/quadrotor_plant.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

GTEST_TEST(QuadrotorPlantTest, DirectFeedthrough) {
  const QuadrotorPlant<double> plant;
  EXPECT_FALSE(plant.HasAnyDirectFeedthrough());
}

GTEST_TEST(QuadrotorPlantTest, ToAutoDiff) {
  const QuadrotorPlant<double> plant;
  EXPECT_TRUE(is_autodiffxd_convertible(plant));
}

GTEST_TEST(QuadrotorPlantTest, ToSymbolic) {
  const QuadrotorPlant<double> plant;
  EXPECT_FALSE(is_symbolic_convertible(plant));
}

// Ensure that DoCalcTimeDerivatives succeeds even if the input port is
// disconnected.
GTEST_TEST(QuadrotorPlantTest, NoInput) {
  const QuadrotorPlant<double> plant;
  auto context = plant.CreateDefaultContext();

  DRAKE_EXPECT_NO_THROW(plant.EvalTimeDerivatives(*context));
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
