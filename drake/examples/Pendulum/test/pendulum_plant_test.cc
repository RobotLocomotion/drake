#include "drake/examples/Pendulum/pendulum_plant.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

GTEST_TEST(PendulumPlantTest, ToAutoDiff) {
  // Construct the plant.
  PendulumPlant<double> plant;
  auto context = plant.CreateDefaultContext();
  // Pretend the state has evolved due to simulation.
  auto& xc = *context->get_mutable_continuous_state_vector();
  xc[0] = 42.0;  // position
  xc[1] = 76.0;  // velocity

  // Transmogrify the plant to autodiff.
  std::unique_ptr<PendulumPlant<AutoDiffXd>> ad_plant =
      systems::System<double>::ToAutoDiffXd<PendulumPlant>(plant);
  ASSERT_NE(nullptr, ad_plant);

  // Construct a new context based on autodiff.
  auto ad_context = ad_plant->CreateDefaultContext();
  ad_context->SetTimeStateAndParametersFrom(*context);
  auto& ad_xc = *ad_context->get_mutable_continuous_state_vector();
  EXPECT_EQ(42.0, ad_xc[0].value());
  EXPECT_EQ(0, ad_xc[0].derivatives().size());
  EXPECT_EQ(76.0, ad_xc[1].value());
  EXPECT_EQ(0, ad_xc[1].derivatives().size());
  // At this point, users could initialize the partials as they pleased.
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
