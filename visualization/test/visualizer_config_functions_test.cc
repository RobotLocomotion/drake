#include "sim/common/drake_visualizer_config_functions.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"

using drake::geometry::DrakeVisualizerParams;
using drake::geometry::Rgba;
using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmBuses;

namespace anzu {
namespace sim {
namespace internal {
namespace {

// For some configuration defaults, we'd like our default config values to
// match up with other default params code in Drake. If these tests ever
// fail, we'll need to revisit whether the Config should change to match
// the Params, or whether we agree they can differ (and remove the test).
GTEST_TEST(DrakeVisualizerConfigTest, Defaults) {
  const DrakeVisualizerConfig config;
  const DrakeVisualizerParams params;
  EXPECT_EQ(config.publish_period, params.publish_period);
  ASSERT_EQ(config.default_illustration_color_rgba.size(), 3);
  EXPECT_EQ(config.default_illustration_color_rgba[0],
            params.default_color.r());
  EXPECT_EQ(config.default_illustration_color_rgba[1],
            params.default_color.g());
  EXPECT_EQ(config.default_illustration_color_rgba[2],
            params.default_color.b());
}

// Tests the mapping from default schema data to geometry params.
GTEST_TEST(DrakeVisualizerConfigFunctionsTest, ParamConversionDefault) {
  const DrakeVisualizerConfig config;
  const std::vector<DrakeVisualizerParams> params =
      ConvertDrakeVisualizerConfigToParams(config);
  ASSERT_EQ(params.size(), 2);
  EXPECT_EQ(params.at(0).role, Role::kIllustration);
  EXPECT_FALSE(params.at(0).show_hydroelastic);
  EXPECT_EQ(params.at(1).role, Role::kProximity);
  EXPECT_TRUE(params.at(1).show_hydroelastic);
  EXPECT_TRUE(params.at(1).use_role_channel_suffix);
  EXPECT_EQ(params.at(0).publish_period, params.at(1).publish_period);
  EXPECT_NE(params.at(0).default_color, params.at(1).default_color);
}

// Tests the mapping from non-default schema data to geometry params.
GTEST_TEST(DrakeVisualizerConfigFunctionsTest, ParamConversionSpecial) {
  DrakeVisualizerConfig config;
  config.publish_period = 0.5;
  config.publish_proximity = false;
  config.default_illustration_color_rgba = Eigen::Vector4d::Constant(0.25);
  const std::vector<DrakeVisualizerParams> params =
      ConvertDrakeVisualizerConfigToParams(config);
  ASSERT_EQ(params.size(), 1);
  EXPECT_EQ(params.at(0).role, Role::kIllustration);
  EXPECT_FALSE(params.at(0).show_hydroelastic);
  EXPECT_EQ(params.at(0).publish_period, 0.5);
  EXPECT_EQ(params.at(0).default_color, Rgba(0.25, 0.25, 0.25, 0.25));
}

// Tests everything disabled.
GTEST_TEST(DrakeVisualizerConfigFunctionsTest, ParamConversionAllDisabled) {
  DrakeVisualizerConfig config;
  config.publish_illustration = false;
  config.publish_proximity = false;
  const std::vector<DrakeVisualizerParams> params =
      ConvertDrakeVisualizerConfigToParams(config);
  EXPECT_EQ(params.size(), 0);
}

// Tests that bad values cause an exception, not a segfault.
GTEST_TEST(DrakeVisualizerConfigFunctionsTest, ParamConversionValidation) {
  DrakeVisualizerConfig bad_color;
  bad_color.default_illustration_color_rgba = {};
  EXPECT_THROW(ConvertDrakeVisualizerConfigToParams(bad_color), std::exception);
}

// Overall acceptance test with everything enabled.
GTEST_TEST(DrakeVisualizerConfigFunctionsTest, ApplyDefault) {
  // We'll monitor which LCM channel names appear.
  DrakeLcm drake_lcm;
  std::set<std::string> observed_channels;
  drake_lcm.SubscribeAllChannels(
      [&observed_channels](std::string_view channel, const void* /* buffer */,
                           int /* size */) {
        observed_channels.emplace(channel);
      });
  LcmBuses lcm_buses;
  lcm_buses.Add("default", &drake_lcm);

  // Add MbP and SG, then the default visualization.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  const DrakeVisualizerConfig config;
  ApplyDrakeVisualizerConfig(config, plant, scene_graph, lcm_buses, &builder);
  Simulator<double> simulator(builder.Build());

  // Simulate for a moment and make sure everything showed up.
  simulator.AdvanceTo(0.25);
  while (drake_lcm.HandleSubscriptions(1) > 0) {}
  EXPECT_THAT(observed_channels, testing::IsSupersetOf({
      "DRAKE_VIEWER_LOAD_ROBOT",
      "DRAKE_VIEWER_LOAD_ROBOT_PROXIMITY",
      "DRAKE_VIEWER_DRAW",
      "DRAKE_VIEWER_DRAW_PROXIMITY",
      "CONTACT_RESULTS"}));
}

// Overall acceptance test with nothing enabled.
GTEST_TEST(DrakeVisualizerConfigFunctionsTest, ApplyNothing) {
  // Disable everything.
  DrakeVisualizerConfig config;
  config.publish_illustration = false;
  config.publish_proximity = false;
  config.publish_contacts = false;

  // We'll fail in case any message is transmitted.
  DrakeLcm drake_lcm;
  drake_lcm.SubscribeAllChannels([](auto...) { GTEST_FAIL(); });
  LcmBuses lcm_buses;
  lcm_buses.Add("default", &drake_lcm);

  // Add MbP and SG, then fully-disabled visualization.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  ApplyDrakeVisualizerConfig(config, plant, scene_graph, lcm_buses, &builder);
  Simulator<double> simulator(builder.Build());

  // Simulate for a moment and make sure nothing showed up.
  simulator.AdvanceTo(0.25);
  drake_lcm.HandleSubscriptions(1);
}

}  // namespace
}  // namespace internal
}  // namespace sim
}  // namespace anzu
