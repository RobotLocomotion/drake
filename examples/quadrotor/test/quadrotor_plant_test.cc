#include "drake/examples/quadrotor/quadrotor_plant.h"

#include <gtest/gtest.h>

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

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake
