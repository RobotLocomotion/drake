#include "drake/examples/quadrotor/quadrotor_plant.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

using symbolic::Expression;
using symbolic::Variable;

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
  EXPECT_TRUE(is_symbolic_convertible(plant));

  auto plant_sym = plant.ToSymbolic();
  auto context_sym = plant_sym->CreateDefaultContext();
  VectorX<Variable> x = symbolic::MakeVectorVariable(12, "x");
  context_sym->get_mutable_continuous_state_vector().SetFromVector(
      x.cast<Expression>());

  VectorX<Expression> derivatives =
      plant_sym->EvalTimeDerivatives(*context_sym).CopyToVector();
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(derivatives[i], x[i + 6]);
  }
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
