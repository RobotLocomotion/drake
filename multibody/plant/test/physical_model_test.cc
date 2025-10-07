#include <memory>
#include <utility>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/dummy_physical_model.h"
#include "drake/multibody/plant/multibody_plant.h"

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
    auto dummy_model = std::make_unique<DummyPhysicalModel<double>>(&plant_);
    dummy_model_ = dummy_model.get();
    plant_.AddDummyModel(std::move(dummy_model));
    // An artificial scenario where the state is added in multiple passes.
    dummy_model_->AppendDiscreteState(dummy_state1());
    dummy_model_->AppendDiscreteState(dummy_state2());
  }

  static VectorXd dummy_state1() {
    return VectorXd::Ones(kState1Dofs) * kState1Value;
  }

  static VectorXd dummy_state2() {
    return VectorXd::Ones(kState2Dofs) * kState2Value;
  }

  MultibodyPlant<double> plant_{kDt};  // A discrete MbP.
  DummyPhysicalModel<double>* dummy_model_{
      nullptr};  // The PhysicalModel under test.
};

// Tests that the state and output ports are properly set up.
TEST_F(PhysicalModelTest, DiscreteStateAndOutputPorts) {
  plant_.Finalize();
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

TEST_F(PhysicalModelTest, DeclareSceneGraphPorts) {
  DRAKE_EXPECT_THROWS_MESSAGE(dummy_model_->GetSceneGraphPortOrThrow(),
                              ".*SceneGraph.*port.*not.*declared.*");
  dummy_model_->DeclareSceneGraphPorts();
  plant_.Finalize();
  const systems::OutputPort<double>* scene_graph_port{nullptr};
  EXPECT_NO_THROW(scene_graph_port = &dummy_model_->GetSceneGraphPortOrThrow());
  auto context = plant_.CreateDefaultContext();
  const VectorXd& output_vector = scene_graph_port->Eval(*context);
  EXPECT_EQ(output_vector, VectorXd::Constant(1, 42.0));
}

// Tests that adding new state after Finalize is not allowed.
TEST_F(PhysicalModelTest, PostFinalizeStateAdditionNotAllowed) {
  plant_.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      dummy_model_->AppendDiscreteState(dummy_state1()),
      "Calls to.*AppendDiscreteState.*after system resources have been "
      "declared are not allowed.");
}

TEST_F(PhysicalModelTest, PostFinalizePortDeclarationNotAllowed) {
  plant_.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      dummy_model_->DeclareSceneGraphPorts(),
      ".*DeclareSceneGraphPorts.* after system resources have been declared.*");
}

GTEST_TEST(PhysicalModelPortTest, DeformableModelDeclareSceneGraphPorts) {
  /* The constructor of MultibodyPlant constructs the deformable model and
   declares its scene graph ports. */
  MultibodyPlant<double> plant(0.01);
  const DeformableModel<double>& deformable_model = plant.deformable_model();
  EXPECT_TRUE(deformable_model.configuration_output_port_index().is_valid());
}

}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
