#include "drake/manipulation/util/zero_force_driver_functions.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace manipulation {
namespace {

using multibody::MultibodyPlant;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

GTEST_TEST(ZeroForceDriverFunctionsTest, SmokeTest) {
  // Create a MultibodyPlant containing a WSG gripper.
  DiagramBuilder<double> builder;
  const auto plant = builder.AddSystem<MultibodyPlant>(0.0);
  const std::string filename = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
  Parser(plant).AddModelFromFile(filename, "gripper");
  plant->Finalize();

  // Apply zero actuation input.
  const ZeroForceDriver config;
  ApplyDriverConfig(config, "gripper", *plant, {}, {}, &builder);

  // Prove that simulation does not crash.
  Simulator<double> simulator(builder.Build());
  simulator.AdvanceTo(0.1);
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
