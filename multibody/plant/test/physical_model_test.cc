#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/dummy_model.h"

namespace drake {
namespace multibody {
namespace internal {
namespace test {
namespace {
using Eigen::VectorXd;
constexpr int kState1Dofs = 2;
constexpr int kState2Dofs = 3;
constexpr double kState1Value = 3.14;
constexpr double kState2Value = 3.15;
constexpr double kDt = 0.1;

class PhysicalModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // TODO(xuchenhan-tri): Add a test with more than one physical model.
    auto dummy_model = std::make_unique<DummyModel<double>>();
    dummy_model_ = dummy_model.get();
    plant_.AddPhysicalModel(std::move(dummy_model));
    // An artificial scenario where the state is added in multiple passes.
    dummy_model_->AppendDiscreteState(dummy_state1());
    dummy_model_->AppendDiscreteState(dummy_state2());
    plant_.Finalize();
  }

  static VectorXd dummy_state1() {
    return VectorXd::Ones(kState1Dofs) * kState1Value;
  }

  static VectorXd dummy_state2() {
    return VectorXd::Ones(kState2Dofs) * kState2Value;
  }

  MultibodyPlant<double> plant_{kDt};         // A discrete MbP.
  DummyModel<double>* dummy_model_{nullptr};  // The PhysicalModel under test.
};

// Tests that the state and output ports are properly set up.
TEST_F(PhysicalModelTest, DiscreteStateAndOutputPorts) {
  auto context = plant_.CreateDefaultContext();
  const VectorXd additional_state =
      dummy_model_->get_vector_output_port().Eval(*context);
  EXPECT_EQ(additional_state.size(), kState1Dofs + kState2Dofs);
  VectorXd expected_state(kState1Dofs + kState2Dofs);
  expected_state.head(kState1Dofs) = dummy_state1();
  expected_state.tail(kState2Dofs) = dummy_state2();
  EXPECT_TRUE(CompareMatrices(additional_state, expected_state));

  // Verifies that the vector and abstract output reports the same state.
  const VectorXd state_through_abstract_port =
      dummy_model_->get_abstract_output_port().Eval<VectorXd>(*context);
  EXPECT_TRUE(CompareMatrices(additional_state, state_through_abstract_port));
}

// Tests that adding new state after Finalize is not allowed.
TEST_F(PhysicalModelTest, PostFinalizeStateAdditionNotAllowed) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      dummy_model_->AppendDiscreteState(dummy_state1()),
      "Calls to 'AppendDiscreteState\\(\\)' after system resources have been "
      "declared are not allowed.");
}
}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
