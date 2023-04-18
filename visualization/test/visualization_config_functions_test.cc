#include "drake/visualization/visualization_config_functions.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"

using drake::geometry::DrakeVisualizerParams;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizerParams;
using drake::geometry::Rgba;
using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::meshcat::ContactVisualizerParams;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmBuses;

namespace drake {
namespace visualization {
namespace internal {
namespace {

// For some configuration defaults, we'd like our default config values to
// match up with other default params code in Drake. If these tests ever
// fail, we'll need to revisit whether the Config should change to match
// the Params, or whether we agree they can differ (and remove the test).
GTEST_TEST(VisualizationConfigTest, Defaults) {
  const VisualizationConfig config;
  const DrakeVisualizerParams drake_params;
  const DrakeVisualizerParams meshcat_params;
  EXPECT_EQ(config.publish_period, drake_params.publish_period);
  EXPECT_EQ(config.default_illustration_color, drake_params.default_color);
  EXPECT_EQ(config.publish_period, meshcat_params.publish_period);
  EXPECT_EQ(config.default_illustration_color, meshcat_params.default_color);
}

// Tests the mapping from default schema data to geometry params.
GTEST_TEST(VisualizationConfigFunctionsTest, ParamConversionDefault) {
  const VisualizationConfig config;

  const std::vector<DrakeVisualizerParams> drake_params =
      ConvertVisualizationConfigToDrakeParams(config);
  ASSERT_EQ(drake_params.size(), 2);

  EXPECT_EQ(drake_params.at(0).role, Role::kIllustration);
  EXPECT_EQ(drake_params.at(0).default_color,
            config.default_illustration_color);
  EXPECT_FALSE(drake_params.at(0).show_hydroelastic);
  EXPECT_FALSE(drake_params.at(0).use_role_channel_suffix);
  EXPECT_EQ(drake_params.at(0).publish_period, config.publish_period);

  EXPECT_EQ(drake_params.at(1).role, Role::kProximity);
  EXPECT_EQ(drake_params.at(1).default_color, config.default_proximity_color);
  EXPECT_TRUE(drake_params.at(1).show_hydroelastic);
  EXPECT_TRUE(drake_params.at(1).use_role_channel_suffix);
  EXPECT_EQ(drake_params.at(1).publish_period, config.publish_period);

  const std::vector<MeshcatVisualizerParams> meshcat_params =
      ConvertVisualizationConfigToMeshcatParams(config);
  ASSERT_EQ(meshcat_params.size(), 2);

  EXPECT_EQ(meshcat_params.at(0).role, Role::kIllustration);
  EXPECT_EQ(meshcat_params.at(0).publish_period, config.publish_period);
  EXPECT_EQ(meshcat_params.at(0).default_color,
            config.default_illustration_color);
  EXPECT_EQ(meshcat_params.at(0).delete_on_initialization_event,
            config.delete_on_initialization_event);
  EXPECT_EQ(meshcat_params.at(0).enable_alpha_slider,
            config.enable_alpha_sliders);
  EXPECT_EQ(meshcat_params.at(0).visible_by_default, true);
  EXPECT_EQ(meshcat_params.at(0).show_hydroelastic, false);

  EXPECT_EQ(meshcat_params.at(1).role, Role::kProximity);
  EXPECT_EQ(meshcat_params.at(1).publish_period, config.publish_period);
  EXPECT_EQ(meshcat_params.at(1).default_color, config.default_proximity_color);
  EXPECT_EQ(meshcat_params.at(1).delete_on_initialization_event,
            config.delete_on_initialization_event);
  EXPECT_EQ(meshcat_params.at(1).enable_alpha_slider,
            config.enable_alpha_sliders);
  EXPECT_EQ(meshcat_params.at(1).visible_by_default, false);
  EXPECT_EQ(meshcat_params.at(1).show_hydroelastic, true);

  const ContactVisualizerParams contact_params =
      ConvertVisualizationConfigToMeshcatContactParams(config);
  EXPECT_EQ(contact_params.publish_period, config.publish_period);
  EXPECT_EQ(contact_params.delete_on_initialization_event,
            config.delete_on_initialization_event);
}

// Tests the mapping from non-default schema data to geometry params.
GTEST_TEST(VisualizationConfigFunctionsTest, ParamConversionSpecial) {
  VisualizationConfig config;
  config.publish_period = 0.5;
  config.publish_proximity = false;
  config.default_illustration_color = Rgba(0.25, 0.25, 0.25, 0.25);
  config.enable_alpha_sliders = true;

  const std::vector<DrakeVisualizerParams> drake_params =
      ConvertVisualizationConfigToDrakeParams(config);
  ASSERT_EQ(drake_params.size(), 1);
  EXPECT_EQ(drake_params.at(0).role, Role::kIllustration);
  EXPECT_FALSE(drake_params.at(0).show_hydroelastic);
  EXPECT_EQ(drake_params.at(0).publish_period, 0.5);
  EXPECT_EQ(drake_params.at(0).default_color, Rgba(0.25, 0.25, 0.25, 0.25));

  const std::vector<MeshcatVisualizerParams> meshcat_params =
      ConvertVisualizationConfigToMeshcatParams(config);
  ASSERT_EQ(meshcat_params.size(), 1);
  EXPECT_EQ(meshcat_params.at(0).role, Role::kIllustration);
  EXPECT_EQ(meshcat_params.at(0).publish_period, 0.5);
  EXPECT_EQ(meshcat_params.at(0).default_color, Rgba(0.25, 0.25, 0.25, 0.25));
  EXPECT_EQ(meshcat_params.at(0).enable_alpha_slider, true);
}

// Tests everything disabled.
GTEST_TEST(VisualizationConfigFunctionsTest, ParamConversionAllDisabled) {
  VisualizationConfig config;
  config.publish_illustration = false;
  config.publish_proximity = false;

  const std::vector<DrakeVisualizerParams> drake_params =
      ConvertVisualizationConfigToDrakeParams(config);
  EXPECT_EQ(drake_params.size(), 0);

  const std::vector<MeshcatVisualizerParams> meshcat_params =
      ConvertVisualizationConfigToMeshcatParams(config);
  EXPECT_EQ(meshcat_params.size(), 0);
}

// Overall acceptance test with everything enabled.
GTEST_TEST(VisualizationConfigFunctionsTest, ApplyDefault) {
  // We'll monitor which LCM channel names appear.
  DrakeLcm drake_lcm;
  std::set<std::string> observed_channels;
  drake_lcm.SubscribeAllChannels(
      [&](std::string_view channel, const void* /* buffer */, int /* size */) {
        observed_channels.emplace(channel);
      });
  LcmBuses lcm_buses;
  lcm_buses.Add("default", &drake_lcm);

  // Add MbP and SG, then the default visualization (ensuring that we can pass
  // an existing Meshcat instance).
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  std::shared_ptr<Meshcat> meshcat = std::make_shared<Meshcat>();
  const VisualizationConfig config;
  ApplyVisualizationConfig(config, &builder, &lcm_buses, &plant, &scene_graph,
                           meshcat);

  // Check that systems that we expect to have been added were actually added.
  for (const auto& name : {
           // For Meldis.
           "drake_visualizer",
           "contact_results_publisher",
           // For Meshcat.
           "meshcat_visualizer",
           "meshcat_contact_visualizer",
       }) {
    SCOPED_TRACE(fmt::format("Checking for a system named like {}", name));
    int count = 0;
    for (const auto* system : builder.GetSystems()) {
      if (system->get_name().find(name) != std::string::npos) {
        ++count;
      }
    }
    EXPECT_GE(count, 1);
  }

  // Simulate for a moment and make sure everything showed up.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.25);
  while (drake_lcm.HandleSubscriptions(1) > 0) {
    // Loop until we're idle.
  }
  // clang-format off
  const auto expected_channels = {
      "DRAKE_VIEWER_LOAD_ROBOT",
      "DRAKE_VIEWER_LOAD_ROBOT_PROXIMITY",
      "DRAKE_VIEWER_DRAW",
      "DRAKE_VIEWER_DRAW_PROXIMITY",
      "CONTACT_RESULTS"
  };
  // clang-format on
  EXPECT_THAT(observed_channels, testing::IsSupersetOf(expected_channels));

  // Check that alpha sliders don't exist by default.
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat->GetSliderValue("illustration α"),
      ".*does not have any slider named illustration.*");
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat->GetSliderValue("proximity α"),
                              ".*does not have any slider named proximity.*");
}

// Overall acceptance test with nothing enabled.
GTEST_TEST(VisualizationConfigFunctionsTest, ApplyNothing) {
  // Disable everything.
  VisualizationConfig config;
  config.publish_illustration = false;
  config.publish_proximity = false;
  config.publish_contacts = false;

  // We'll fail in case any message is transmitted.
  DrakeLcm drake_lcm;
  drake_lcm.SubscribeAllChannels([](auto...) {
    GTEST_FAIL();
  });
  LcmBuses lcm_buses;
  lcm_buses.Add("default", &drake_lcm);

  // Add MbP and SG, then fully-disabled visualization.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  ApplyVisualizationConfig(config, &builder, &lcm_buses, &plant, &scene_graph);

  // Check that no meshcat object is created if there is nothing to visualize.
  int meshcat_count = 0;
  for (const auto* system : builder.GetSystems()) {
    const std::string& name = system->get_name();
    if (name.find("MeshcatVisualizer") != std::string::npos) {
      ++meshcat_count;
    }
  }
  EXPECT_EQ(meshcat_count, 0);

  Simulator<double> simulator(builder.Build());

  // Simulate for a moment and make sure nothing showed up.
  simulator.AdvanceTo(0.25);
  drake_lcm.HandleSubscriptions(1);
}

// Check that a Meshcat instance is created when none is passed.
GTEST_TEST(VisualizationConfigFunctionsTest, NoMeshcat) {
  DrakeLcm drake_lcm;
  LcmBuses lcm_buses;
  lcm_buses.Add("default", &drake_lcm);

  // Add MbP and SG, then the default visualization.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  const VisualizationConfig config;
  ApplyVisualizationConfig(config, &builder, &lcm_buses, &plant, &scene_graph);

  int meshcat_count = 0;
  for (const auto* system : builder.GetSystems()) {
    const std::string& name = system->get_name();
    if (name.find("meshcat_visualizer") != std::string::npos) {
      ++meshcat_count;
    }
  }
  EXPECT_EQ(meshcat_count, 2);
}

// Check that turning on the alpha sliders functions as expected.
GTEST_TEST(VisualizationConfigFunctionsTest, AlphaSliders) {
  DrakeLcm drake_lcm;
  LcmBuses lcm_buses;
  lcm_buses.Add("default", &drake_lcm);

  // Add MbP and SG, then the default visualization.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  std::shared_ptr<Meshcat> meshcat = std::make_shared<Meshcat>();
  VisualizationConfig config;
  config.enable_alpha_sliders = true;
  ApplyVisualizationConfig(config, &builder, &lcm_buses, &plant, &scene_graph,
                           meshcat);

  // Check that alpha sliders exist.
  meshcat->GetSliderValue("illustration α");
  meshcat->GetSliderValue("proximity α");
}

// The AddDefault... sugar shouldn't crash.
GTEST_TEST(VisualizationConfigFunctionsTest, AddDefault) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  AddDefaultVisualization(&builder);
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.25);
}

// The AddDefault... sugar should respect our passed-in Meshcat.
GTEST_TEST(VisualizationConfigFunctionsTest, AddDefaultWithMeshcat) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  std::shared_ptr<Meshcat> meshcat = std::make_shared<Meshcat>();
  EXPECT_EQ(meshcat.use_count(), 1);
  AddDefaultVisualization(&builder, meshcat);
  Simulator<double> simulator(builder.Build());
  // Our meshcat was actually used.
  EXPECT_GE(meshcat.use_count(), 2);
  // Nothing crashes.
  simulator.AdvanceTo(0.25);
}

// A missing plant causes an exception.
GTEST_TEST(VisualizationConfigFunctionsTest, NoPlant) {
  DiagramBuilder<double> builder;
  DRAKE_EXPECT_THROWS_MESSAGE(AddDefaultVisualization(&builder),
                              ".*does not contain.*plant.*");
}

// A missing scene_graph causes an exception.
GTEST_TEST(VisualizationConfigFunctionsTest, NoSceneGraph) {
  DiagramBuilder<double> builder;
  auto* plant = builder.AddSystem<MultibodyPlant<double>>(0.0);
  plant->set_name("plant");
  plant->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(AddDefaultVisualization(&builder),
                              ".*does not contain.*scene_graph.*");
}

// Type confusion causes an exception.
GTEST_TEST(VisualizationConfigFunctionsTest, WrongSystemTypes) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.set_name("scene_graph");
  scene_graph.set_name("plant");
  DRAKE_EXPECT_THROWS_MESSAGE(AddDefaultVisualization(&builder),
                              ".*not cast.*");
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Make sure the deprecated overload doesn't crash.
GTEST_TEST(VisualizationConfigFunctionsTest, DeprecatedApply) {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();

  // Call the deprecated overload (with const pointers).
  const VisualizationConfig config;
  const MultibodyPlant<double>* const const_plant = &plant;
  const SceneGraph<double>* const const_scene_graph = &scene_graph;
  ApplyVisualizationConfig(config, &builder, nullptr, const_plant,
                           const_scene_graph);

  // Simulate for a moment and make sure nothing crashes.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.25);
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
