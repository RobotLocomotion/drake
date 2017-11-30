#include "drake/examples/pendulum/pendulum_plant.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

GTEST_TEST(PendulumPlantTest, ToAutoDiff) {
  // Construct the plant.
  PendulumPlant<double> plant;
  auto context = plant.CreateDefaultContext();
  // Pretend the state has evolved due to simulation.
  auto& xc = context->get_mutable_continuous_state_vector();
  xc[0] = 42.0;  // position
  xc[1] = 76.0;  // velocity

  // Transmogrify the plant to autodiff.
  std::unique_ptr<PendulumPlant<AutoDiffXd>> ad_plant =
      systems::System<double>::ToAutoDiffXd(plant);
  ASSERT_NE(nullptr, ad_plant);

  // Construct a new context based on autodiff.
  auto ad_context = ad_plant->CreateDefaultContext();
  ad_context->SetTimeStateAndParametersFrom(*context);
  auto& ad_xc = ad_context->get_mutable_continuous_state_vector();
  EXPECT_EQ(42.0, ad_xc[0].value());
  EXPECT_EQ(0, ad_xc[0].derivatives().size());
  EXPECT_EQ(76.0, ad_xc[1].value());
  EXPECT_EQ(0, ad_xc[1].derivatives().size());
  // At this point, users could initialize the partials as they pleased.
}

GTEST_TEST(PendulumPlantTest, DirectFeedthrough) {
  PendulumPlant<double> plant;
  EXPECT_FALSE(plant.HasAnyDirectFeedthrough());
}

GTEST_TEST(PendulumPlantTest, CalcTotalEnergy) {
  PendulumPlant<double> plant;
  const auto context = plant.CreateDefaultContext();

  const auto params = dynamic_cast<const PendulumParams<double>*>(
      &context->get_numeric_parameter(0));
  EXPECT_TRUE(params);

  auto* state = dynamic_cast<PendulumState<double>*>(
      &context->get_mutable_continuous_state_vector());
  EXPECT_TRUE(state);

  const double kTol = 1e-6;
  // Energy at the bottom is -mgl.
  state->set_theta(0.0);
  state->set_thetadot(0.0);
  EXPECT_NEAR(plant.CalcTotalEnergy(*context),
              -params->mass() * params->gravity() * params->length(), kTol);

  // Energy at the top is mgl.
  state->set_theta(M_PI);
  state->set_thetadot(0.0);
  EXPECT_NEAR(plant.CalcTotalEnergy(*context),
              params->mass() * params->gravity() * params->length(), kTol);

  // Energy at horizontal is 1/2 m v^2.
  state->set_theta(M_PI_2);
  state->set_thetadot(1.0);
  EXPECT_NEAR(plant.CalcTotalEnergy(*context),
              0.5 * params->mass() * std::pow(params->length(), 2), kTol);
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
