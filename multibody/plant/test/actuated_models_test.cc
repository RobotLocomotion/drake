#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::VectorXd;
using multibody::Parser;
using systems::Context;

namespace multibody {

// MultibodyPlant friend class used to provide access to private methods for
// testing purposes.
class MultibodyPlantTester {
 public:
  static VectorXd AssembleActuationInput(const MultibodyPlant<double>& plant,
                                         const Context<double>& context) {
    return plant.AssembleActuationInput(context);
  }

  static VectorXd AssembleDesiredStateInput(const MultibodyPlant<double>& plant,
                                            const Context<double>& context) {
    return plant.AssembleDesiredStateInput(context);
  }
};

namespace {

// This fixture loads a MultibodyPlant model of a KUKA Iiiwa arm with a Schunk
// gripper.
class ActuatedIiiwaArmTest : public ::testing::Test {
 public:
  enum class ModelConfiguration {
    kArmIsControlled,
    kArmIsNotControlled,
    kArmIsPartiallyControlled,
  };

  // - arm not controlled
  // - arm controlled
  // - arm partially controlled
  void SetUpModel(
      ModelConfiguration model_config = ModelConfiguration::kArmIsNotControlled,
      bool is_discrete = true) {
    const char kArmSdfPath[] =
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf";

    const char kWsg50SdfPath[] =
        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    // Make a discrete model.
    const double update_period = is_discrete ? 0.01 : 0.0;
    plant_ = std::make_unique<MultibodyPlant<double>>(update_period);
    // Use the SAP solver. Thus far only SAP support the modeling of PD
    // controllers.
    plant_->set_discrete_contact_solver(DiscreteContactSolver::kSap);

    Parser parser(plant_.get());

    // Add the arm.
    arm_model_ = parser.AddModels(FindResourceOrThrow(kArmSdfPath)).at(0);

    // Add the gripper.
    gripper_model_ = parser.AddModels(FindResourceOrThrow(kWsg50SdfPath)).at(0);

    // A model of a (non-actuated) plate.
    box_model_ = parser
                       .AddModels(FindResourceOrThrow(
                           "drake/multibody/models/box.urdf"))
                       .at(0);

    const auto& base_body = plant_->GetBodyByName("iiwa_link_0", arm_model_);
    const auto& end_effector = plant_->GetBodyByName("iiwa_link_7", arm_model_);
    const auto& gripper_body = plant_->GetBodyByName("body", gripper_model_);
    plant_->WeldFrames(plant_->world_frame(), base_body.body_frame());
    plant_->WeldFrames(end_effector.body_frame(), gripper_body.body_frame());

    // Set PD controllers for the gripper.
    SetGripperModel();

    // Arm actuators.
    std::vector<JointActuatorIndex> arm_actuators;
    for (JointActuatorIndex actuator_index(0);
         actuator_index < plant_->num_actuators(); ++actuator_index) {
      if (plant_->get_joint_actuator(actuator_index).model_instance() ==
          arm_model_) {
        arm_actuators.push_back(actuator_index);
      }
    }

    // Set PD controllers for the arm, depending on the desired configuration.
    if (model_config == ModelConfiguration::kArmIsControlled) {
      // Define PD controllers for the arm.
      for (JointActuatorIndex actuator_index : arm_actuators) {
        JointActuator<double>& actuator =
            plant_->get_mutable_joint_actuator(actuator_index);
        actuator.set_controller_gains({kProportionalGain_, kDerivativeGain_});
      }
    } else if (model_config == ModelConfiguration::kArmIsPartiallyControlled) {
      // Add PD control only on a subset of actuators.
      auto& actuator1 = plant_->get_mutable_joint_actuator(arm_actuators[1]);
      actuator1.set_controller_gains({kProportionalGain_, kDerivativeGain_});
      auto& actuator3 = plant_->get_mutable_joint_actuator(arm_actuators[3]);
      actuator3.set_controller_gains({kProportionalGain_, kDerivativeGain_});
    }

    plant_->Finalize();

    context_ = plant_->CreateDefaultContext();
  }

  void SetGripperModel() {
    for (JointActuatorIndex actuator_index(0);
         actuator_index < plant_->num_actuators(); ++actuator_index) {
      JointActuator<double>& actuator =
          plant_->get_mutable_joint_actuator(actuator_index);
      if (actuator.model_instance() == gripper_model_) {
        actuator.set_controller_gains({kProportionalGain_, kDerivativeGain_});
      }
    }
  }

 protected:
  const int kKukaNumPositions_{7};
  const int kGripperNumPositions_{2};
  const double kProportionalGain_{10000.0};
  const double kDerivativeGain_{100.0};
  std::unique_ptr<MultibodyPlant<double>> plant_;
  ModelInstanceIndex arm_model_;
  ModelInstanceIndex gripper_model_;
  ModelInstanceIndex box_model_;
  std::unique_ptr<Context<double>> context_;
};

TEST_F(ActuatedIiiwaArmTest, JointActuatorApis) {
  SetUpModel();
  for (JointActuatorIndex actuator_index :
       plant_->GetJointActuatorIndices(gripper_model_)) {
    const auto& actuator = plant_->get_joint_actuator(actuator_index);
    ASSERT_TRUE(actuator.has_controller());
    const PdControllerGains& gains = actuator.get_controller_gains();
    EXPECT_EQ(gains.p, kProportionalGain_);
    EXPECT_EQ(gains.d, kDerivativeGain_);
  }
}

TEST_F(ActuatedIiiwaArmTest, GetActuationInputPort) {
  SetUpModel();

  EXPECT_NO_THROW(plant_->get_actuation_input_port(arm_model_));
  EXPECT_NO_THROW(plant_->get_actuation_input_port(gripper_model_));

  // We always have an actuation input port, even for unactuated models. In this
  // case the port is zero sized.
  EXPECT_EQ(plant_->get_actuation_input_port(box_model_).size(), 0);

  // Invalid model index throws.
  const ModelInstanceIndex invalid_index(plant_->num_model_instances() + 10);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->get_actuation_input_port(invalid_index),
      ".* get_actuation_input_port\\(\\): condition 'model_instance < "
      "num_model_instances\\(\\)' failed.");
}

TEST_F(ActuatedIiiwaArmTest, GetDesiredStatePort) {
  SetUpModel(ModelConfiguration::kArmIsNotControlled);

  EXPECT_NO_THROW(plant_->get_desired_state_input_port(arm_model_));
  EXPECT_NO_THROW(plant_->get_desired_state_input_port(gripper_model_));

  // For consistency with actuation input ports, all model instances have a
  // desired state input port. If the model instance has no PD controllers, this
  // port will have zero size.
  EXPECT_EQ(plant_->get_desired_state_input_port(arm_model_).size(),
            0);  // The arm is not PD controlled.
  EXPECT_EQ(plant_->get_desired_state_input_port(box_model_).size(),
            0);  // The free floating box has no actuators.

  // Invalid model index throws.
  const ModelInstanceIndex invalid_index(plant_->num_model_instances() + 10);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->get_desired_state_input_port(invalid_index),
      ".* get_desired_state_input_port\\(\\): condition 'model_instance < "
      "num_model_instances\\(\\)' failed.");
}

// Verify that MultibodyPlant::AssembleActuationInput() throws an exception if
// the actuation input port for a model instance without PD controllers is not
// connected.
TEST_F(ActuatedIiiwaArmTest, AssembleActuationInput_ActuationInputRequired) {
  SetUpModel();

  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyPlantTester::AssembleActuationInput(*plant_, *context_),
      "Actuation input port for model instance iiwa14 must be connected or PD "
      "gains must be specified for each actuator.");
}

// Verify that actuation port is not required to be connected if the model is PD
// controlled.
TEST_F(ActuatedIiiwaArmTest, AssembleActuationInput_NotRequiredIfPdControlled) {
  SetUpModel();

  // The actuation input port for the arm is required to be connected.
  plant_->get_actuation_input_port(arm_model_)
      .FixValue(context_.get(), VectorXd::Zero(kKukaNumPositions_));

  // The actuation input port for the gripper is not required to be connected
  // since its actuators are PD controlled.
  EXPECT_NO_THROW(
      MultibodyPlantTester::AssembleActuationInput(*plant_, *context_));
}

// Verify that MultibodyPlant::AssembleDesiredStateInput() throws an exception
// when not all actuators in a model instance are PD controlled. Once a PD
// controller is defined in a model instance, all actuators must use PD control.
TEST_F(ActuatedIiiwaArmTest,
       AssembleDesiredStateInput_ThrowsIfPartiallyPDControlled) {
  SetUpModel(ModelConfiguration::kArmIsPartiallyControlled);

  // The gripper has controllers in all of its actuators and thus it is required
  // to be connected.
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), VectorXd::Zero(2 * kGripperNumPositions_));

  // We now verify AssembleDesiredStateInput() throws for the right reason.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyPlantTester::AssembleDesiredStateInput(*plant_, *context_),
      "Model iiwa14 is partially PD controlled. .*");
}

TEST_F(ActuatedIiiwaArmTest,
       AssembleDesiredStateInput_ThrowsIfDesiredStateNotConnected) {
  SetUpModel();

  // The input port for desired states for the gripper is required to be
  // connected.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyPlantTester::AssembleDesiredStateInput(*plant_, *context_),
      "Desired state input port for model instance Schunk_Gripper not "
      "connected.");
}

// Verify the assembly of desired states for a plant with a single PD controlled
// model instance.
TEST_F(ActuatedIiiwaArmTest,
       AssembleDesiredStateInput_VerifyAssemblyWithOneModel) {
  SetUpModel();

  const VectorXd gripper_xd = (VectorXd(4) << 1., 2., 3, 4.).finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);
  const VectorXd full_xd =
      MultibodyPlantTester::AssembleDesiredStateInput(*plant_, *context_);

  const int nu = plant_->num_actuated_dofs();
  // AssembleDesiredStateInput will always return a vector of size 2 * nu. It
  // fill in values for those models with PD control and set all other entries
  // to zero. In this case, entries corresponding to the arm are expected to be
  // zero. All qd values go first, followed by vd values.
  const int arm_nu = plant_->num_actuated_dofs(arm_model_);
  VectorXd expected_xd =
      (VectorXd(2 * nu) << VectorXd::Zero(arm_nu), gripper_xd.head<2>(),
       VectorXd::Zero(arm_nu), gripper_xd.tail<2>())
          .finished();

  EXPECT_EQ(full_xd, expected_xd);
}

// Verify the assembly of desired states for a plant with two PD controlled
// model instances.
TEST_F(ActuatedIiiwaArmTest,
       AssembleDesiredStateInput_VerifyAssemblyWithTwoModels) {
  SetUpModel(ModelConfiguration::kArmIsControlled);

  // Fixed desired state input ports to known values.
  // Both arm and gripper are required to be connected.
  const VectorXd gripper_xd = (VectorXd(4) << 1., 2., 3, 4.).finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);
  const VectorXd arm_xd = VectorXd::LinSpaced(14, 1.0, 14.0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  const VectorXd full_xd =
      MultibodyPlantTester::AssembleDesiredStateInput(*plant_, *context_);
  // Desired states must be assembled according to model instance order,
  // therefore in this case arm first followed by gripper. All qd values go
  // first, followed by vd values.
  const int nu = plant_->num_actuated_dofs();
  const VectorXd expected_xd =
      (VectorXd(2 * nu) << arm_xd.head<7>(), gripper_xd.head<2>(),
       arm_xd.tail<7>(), gripper_xd.tail<2>())
          .finished();

  EXPECT_EQ(full_xd, expected_xd);
}

TEST_F(ActuatedIiiwaArmTest,
       PdControlledActuatorsOnlySupportedForDiscreteModels) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      SetUpModel(ModelConfiguration::kArmIsControlled, false),
      "Continuous model with PD controlled joint actuators. This feature is "
      "only supported for discrete models. Refer to MultibodyPlant's "
      "documentation for further details.");
}

}  // namespace
}  // namespace multibody
}  // namespace drake
