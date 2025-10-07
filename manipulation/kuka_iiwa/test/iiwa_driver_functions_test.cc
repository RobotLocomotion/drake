#include "drake/manipulation/kuka_iiwa/iiwa_driver_functions.h"

#include <map>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/drake_lcm_params.h"
#include "drake/manipulation/kuka_iiwa/iiwa_driver.h"
#include "drake/manipulation/util/zero_force_driver_functions.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_config_functions.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using lcm::DrakeLcmParams;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using multibody::parsing::LoadModelDirectives;
using multibody::parsing::ModelDirectives;
using multibody::parsing::ModelInstanceInfo;
using systems::DiagramBuilder;
using systems::Simulator;
using systems::System;
using systems::lcm::ApplyLcmBusConfig;
using systems::lcm::LcmBuses;

/* A smoke test to apply simulated Iiwa driver with different driver configs.
 Specifically, whether a correct arm and hand model names are provided is
 tested. */
GTEST_TEST(IiwaDriverFunctionsTest, ApplyDriverConfig) {
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant =
      AddMultibodyPlant(MultibodyPlantConfig{}, &builder);
  const ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow("drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"));
  Parser parser{&plant};
  std::vector<ModelInstanceInfo> models_from_directives =
      multibody::parsing::ProcessModelDirectives(directives, &parser);
  plant.Finalize();

  std::map<std::string, ModelInstanceInfo> models_from_directives_map;
  for (const auto& info : models_from_directives) {
    models_from_directives_map.emplace(info.model_name, info);
  }

  const std::map<std::string, DrakeLcmParams> lcm_bus_config = {
      {"default", {}}};
  const LcmBuses lcm_buses = ApplyLcmBusConfig(lcm_bus_config, &builder);

  // Supply incorrect arm model name should throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ApplyDriverConfig(IiwaDriver{}, "arm", plant, models_from_directives_map,
                        lcm_buses, &builder),
      "IiwaDriver could not find arm model.*");
  // Supply an Iiwa driver with an incorrect `hand_model_name` should throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ApplyDriverConfig(IiwaDriver{.hand_model_name = "hand"}, "iiwa7", plant,
                        models_from_directives_map, lcm_buses, &builder),
      "IiwaDriver could not find hand model.*");

  // Supply incorrect frame names for combining hand and arm.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ApplyDriverConfig(IiwaDriver{.arm_child_frame_name = "nope"}, "iiwa7",
                        plant, models_from_directives_map, lcm_buses, &builder),
      ".*no Frame named 'nope'.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ApplyDriverConfig(IiwaDriver{.hand_model_name = "schunk_wsg",
                                   .gripper_parent_frame_name = "haha"},
                        "iiwa7", plant, models_from_directives_map, lcm_buses,
                        &builder),
      ".*no Frame named 'haha'.*");

  const ZeroForceDriver wsg_driver;
  const IiwaDriver iiwa_driver{.hand_model_name = "schunk_wsg"};
  ApplyDriverConfig(wsg_driver, "schunk_wsg", plant, {}, {}, &builder);
  ApplyDriverConfig(iiwa_driver, "iiwa7", plant, models_from_directives_map,
                    lcm_buses, &builder);

  // Prove that simulation does not crash.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.1);
}

GTEST_TEST(IiwaDriverFunctionsTest, RenamedHandWeldFrames) {
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant =
      AddMultibodyPlant(MultibodyPlantConfig{}, &builder);
  // Load a modified model with a different frame name for the gripper
  // weld. The frame "hand_parent" is added via directives; the SDFormat file
  // for the arm is a stock version.
  const ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow("drake/manipulation/kuka_iiwa/test/"
                          "iiwa7_wsg_renamed_weld_frames.dmd.yaml"));
  Parser parser{&plant};
  std::vector<ModelInstanceInfo> models_from_directives =
      multibody::parsing::ProcessModelDirectives(directives, &parser);
  plant.Finalize();

  std::map<std::string, ModelInstanceInfo> models_from_directives_map;
  for (const auto& info : models_from_directives) {
    models_from_directives_map.emplace(info.model_name, info);
  }

  const std::map<std::string, DrakeLcmParams> lcm_bus_config = {
      {"default", {}}};
  const LcmBuses lcm_buses = ApplyLcmBusConfig(lcm_bus_config, &builder);

  // Configure drivers with non-default frame names for the welds. The
  // `arm_child_frame_name` is constrained by the implementation of
  // MakeArmControllerModel to one that was defined in the underlying (SDFormat
  // in this case) model file. The `gripper_parent_frame_name` can come from
  // directives, since it gets looked up in the simulation plant, and
  // more-or-less semantically reconstructed in the controller plant.
  const ZeroForceDriver wsg_driver;
  const IiwaDriver iiwa_driver{.hand_model_name = "schunk_wsg",
                               .arm_child_frame_name = "__model__",
                               .gripper_parent_frame_name = "hand_parent"};
  ApplyDriverConfig(wsg_driver, "schunk_wsg", plant, {}, {}, &builder);
  ApplyDriverConfig(iiwa_driver, "iiwa7", plant, models_from_directives_map,
                    lcm_buses, &builder);

  // Prove that simulation does not crash.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.1);
}

/* Confirm that the user can opt-out of LCM. */
GTEST_TEST(IiwaDriverFunctionsTest, ApplyDriverConfigNoLcm) {
  // Prepare the plant.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant =
      AddMultibodyPlant(MultibodyPlantConfig{}, &builder);
  const ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow("drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"));
  Parser parser{&plant};
  std::vector<ModelInstanceInfo> models_from_directives =
      multibody::parsing::ProcessModelDirectives(directives, &parser);
  plant.Finalize();
  std::map<std::string, ModelInstanceInfo> models_from_directives_map;
  for (const auto& info : models_from_directives) {
    models_from_directives_map.emplace(info.model_name, info);
  }

  // Use nullopt for the LCM bus.
  const std::map<std::string, std::optional<DrakeLcmParams>> lcm_bus_config = {
      {"default", std::nullopt}};
  const LcmBuses lcm_buses = ApplyLcmBusConfig(lcm_bus_config, &builder);

  // Add the driver.
  const IiwaDriver iiwa_driver;
  ApplyDriverConfig(iiwa_driver, "iiwa7", plant, models_from_directives_map,
                    lcm_buses, &builder);

  // Check that no unwanted systems were added.
  auto diagram = builder.Build();
  std::vector<std::string> names;
  for (const System<double>* system : diagram->GetSystems()) {
    names.push_back(system->get_name());
  }
  const std::vector<std::string> expected{
      // From AddMultibodyPlant.
      "plant",
      "scene_graph",
      // From ApplyDriverConfig.
      "iiwa7",
      // TODO(jwnimmer-tri) Ideally this SharedPtrSystem would live within the
      // SimIiwaDriver, but for the moment it's a sibling instead of a child.
      "iiwa7_controller_plant",
      // This one SharedPtrSystem is the only remnant of LCM that gets added.
      // It is a DrakeLcmInterface that is never pumped (note that there is
      // no LcmInterfaceSystem anywhere in this list of `expected` systems.
      "DrakeLcm(bus_name=default)",
  };
  EXPECT_THAT(names, testing::UnorderedElementsAreArray(expected));
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
