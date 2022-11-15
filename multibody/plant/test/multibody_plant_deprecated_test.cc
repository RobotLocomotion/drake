#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
  Parser(plant.get()).AddModels(FindResourceOrThrow(
      "drake/multibody/plant/test/split_pendulum.sdf"));
  plant->Finalize();
  EXPECT_EQ(plant->num_model_instances(), 3);

  // Check that deprecated output ports have the appropriate sizes.
  const auto& all_state_port_old =
      plant->GetOutputPort("continuous_state");
  const auto& default_state_port_old =
      plant->GetOutputPort("DefaultModelInstance_continuous_state");
  const auto& pendulum_state_port_old =
      plant->GetOutputPort("SplitPendulum_continuous_state");
  EXPECT_EQ(all_state_port_old.size(), 6);
  EXPECT_EQ(default_state_port_old.size(), 4);
  EXPECT_EQ(pendulum_state_port_old.size(), 2);

  // Check that the deprecated output ports match the non-deprecated ones.
  auto context = plant->CreateDefaultContext();
  plant->SetPositions(context.get(), Eigen::Vector3d{0.1, 0.2, 0.3});
  plant->SetVelocities(context.get(), Eigen::Vector3d{0.01, 0.02, 0.03});
  const auto& all_state_port_new =
      plant->GetOutputPort("state");
  const auto& default_state_port_new =
      plant->GetOutputPort("DefaultModelInstance_state");
  const auto& pendulum_state_port_new =
      plant->GetOutputPort("SplitPendulum_state");
  EXPECT_TRUE(CompareMatrices(all_state_port_old.Eval(*context),
                              all_state_port_new.Eval(*context)));
  EXPECT_TRUE(CompareMatrices(default_state_port_old.Eval(*context),
                              default_state_port_new.Eval(*context)));
  EXPECT_TRUE(CompareMatrices(pendulum_state_port_old.Eval(*context),
                              pendulum_state_port_new.Eval(*context)));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
