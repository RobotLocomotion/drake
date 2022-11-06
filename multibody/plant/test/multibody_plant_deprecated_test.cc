#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace {

using benchmarks::acrobot::AcrobotParameters;
using benchmarks::acrobot::MakeAcrobotPlant;

// Checks the deprecated output port behavior.
GTEST_TEST(MultibodyPlant, DeprecatedOutputNames) {
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, /* finalize = */ false);
  Parser(plant.get()).AddModelFromFile(FindResourceOrThrow(
      "drake/multibody/plant/test/split_pendulum.sdf"));
  plant->Finalize();
  EXPECT_EQ(plant->num_model_instances(), 3);

  // Check that the input/output ports have the appropriate geometry.
  const auto& all_state_port =
      plant->GetOutputPort("continuous_state");
  const auto& default_state_port =
      plant->GetOutputPort("DefaultModelInstance_continuous_state");
  const auto& pendulum_state_port =
      plant->GetOutputPort("SplitPendulum_continuous_state");
  EXPECT_EQ(all_state_port.size(), 6);
  EXPECT_EQ(default_state_port.size(), 4);
  EXPECT_EQ(pendulum_state_port.size(), 2);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
