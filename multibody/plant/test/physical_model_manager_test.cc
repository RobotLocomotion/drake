#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/dummy_model_manager.h"

namespace drake {
namespace multibody {
namespace internal {
namespace test {
namespace {
using Eigen::VectorXd;
constexpr int kNumAdditionalDofs = 9;
constexpr double kDummyStateValue = 3.15;
constexpr double kDt = 0.1;

class PhysicalModelManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    manager_ =
        &plant_.AddExternalModel(std::make_unique<DummyModelManager>(&plant_));
    manager_->AppendDiscreteState(dummy_discrete_state());
    manager_->Finalize();
    plant_.Finalize();
  }

  static VectorXd dummy_discrete_state() {
    return VectorXd::Ones(kNumAdditionalDofs) * kDummyStateValue;
  }

  MultibodyPlant<double> plant_{kDt};    // A discrete MbP.
  DummyModelManager* manager_{nullptr};  // The manager under test.
};

TEST_F(PhysicalModelManagerTest, DiscreteStateAndOutputPorts) {
  auto context = plant_.CreateDefaultContext();
  const VectorXd additional_state =
      manager_->get_vector_output_port().Eval(*context);
  EXPECT_EQ(additional_state.size(), kNumAdditionalDofs);
  EXPECT_TRUE(CompareMatrices(additional_state, dummy_discrete_state()));

  // Verifies that the vector and abstract output reports the same state.
  const VectorXd state_through_abstract_port =
      manager_->get_abstract_output_port().Eval<VectorXd>(*context);
  EXPECT_TRUE(CompareMatrices(additional_state, state_through_abstract_port));
}
}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
