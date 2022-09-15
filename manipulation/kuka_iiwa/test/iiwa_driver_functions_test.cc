#include "drake/manipulation/kuka_iiwa/iiwa_driver_functions.h"

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
  const LcmBuses lcm_buses =
      systems::lcm::ApplyLcmBusConfig(lcm_bus_config, &builder);

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

  const ZeroForceDriver wsg_driver;
  const IiwaDriver iiwa_driver{.hand_model_name = "schunk_wsg"};
  ApplyDriverConfig(wsg_driver, "schunk_wsg", plant, {}, {}, &builder);
  ApplyDriverConfig(iiwa_driver, "iiwa7", plant, models_from_directives_map,
                    lcm_buses, &builder);

  // Prove that simulation does not crash.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.1);
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
