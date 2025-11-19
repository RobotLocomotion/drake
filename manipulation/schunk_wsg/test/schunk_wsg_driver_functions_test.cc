#include "drake/manipulation/schunk_wsg/schunk_wsg_driver_functions.h"

#include <map>
#include <string>

#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm_params.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_driver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_config_functions.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

using lcm::DrakeLcmParams;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;
using systems::lcm::LcmBuses;

/* A smoke test to apply simulated wsg driver with default driver config. */
GTEST_TEST(SchunkWsgDriverFunctionsTest, ApplyDriverConfig) {
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant =
      AddMultibodyPlant(MultibodyPlantConfig{}, &builder);
  const std::string url =
      "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf";
  const ModelInstanceIndex schunk_wsg =
      Parser(&plant).AddModelsFromUrl(url).at(0);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("body", schunk_wsg));
  plant.Finalize();

  const std::map<std::string, DrakeLcmParams> lcm_bus_config = {
      {"default", {}}};
  const LcmBuses lcm_buses =
      systems::lcm::ApplyLcmBusConfig(lcm_bus_config, &builder);

  const SchunkWsgDriver schunk_wsg_driver;
  const std::string schunk_wsg_name = plant.GetModelInstanceName(schunk_wsg);
  ApplyDriverConfig(schunk_wsg_driver, schunk_wsg_name, plant, {}, lcm_buses,
                    &builder);

  // Prove that simulation does not crash.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.1);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
