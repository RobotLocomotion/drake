#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using drake::multibody::benchmarks::pendulum::MakePendulumPlant;
using drake::multibody::benchmarks::pendulum::PendulumParameters;

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(MultibodyPlantSymbolicTest, Pendulum) {
  // Make the double-valued system.
  PendulumParameters params;
  const double m = params.m();
  const double g = params.g();
  const double l = params.l();
  const double damping = params.damping();
  auto pendulum_double = MakePendulumPlant(params);
  ASSERT_TRUE(is_symbolic_convertible(*pendulum_double));

  // Make the symbolic system.
  auto dut = MultibodyPlant<double>::ToSymbolic(*pendulum_double);
  auto context = dut->CreateDefaultContext();

  // Set the input and state to variables.
  using T = symbolic::Expression;
  const symbolic::Variable tau("tau");
  const symbolic::Variable theta("theta");
  const symbolic::Variable thetadot("thetadot");
  dut->get_actuation_input_port().FixValue(context.get(), T(tau));
  dut->SetPositionsAndVelocities(context.get(), Vector2<T>(theta, thetadot));

  // Check the symbolic derivatives.
  const auto& derivatives = dut->EvalTimeDerivatives(*context);
  ASSERT_EQ(derivatives.size(), 2);
  EXPECT_PRED2(symbolic::test::ExprEqual, derivatives[0], thetadot);

  // Drake's expression for thetaddot in derivatives[1] depends on which Eigen
  // solver is used for factorizing the "hinge matrix", e.g., a standard LLᵀ
  // Cholesky matrix decomposition or a LDLT matrix decomposition.
  // Info: https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
  // Use the Expand() command to faciliate comparison of Drake's symbolic
  // result with the expected symbolic result.
  const T expected_thetaddot =
      (tau - m * g * l * sin(theta) - damping * thetadot) / (l * l);
  EXPECT_PRED2(symbolic::test::ExprEqual, derivatives[1].Expand(),
               expected_thetaddot.Expand());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
