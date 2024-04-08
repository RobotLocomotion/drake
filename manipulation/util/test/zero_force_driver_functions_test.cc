#include "drake/manipulation/util/zero_force_driver_functions.h"

#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace manipulation {
namespace {

using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

GTEST_TEST(ZeroForceDriverFunctionsTest, SmokeTest) {
  // Create a MultibodyPlant containing a WSG gripper.
  DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant =
      AddMultibodyPlant(MultibodyPlantConfig{}, &builder);
  Parser(&plant).AddModelsFromUrl(
      "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf");
  plant.Finalize();

  // Apply zero actuation input.
  const ZeroForceDriver config;
  ApplyDriverConfig(config, "Schunk_Gripper", plant, {}, {}, &builder);

  // Prove that simulation does not crash.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.1);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
