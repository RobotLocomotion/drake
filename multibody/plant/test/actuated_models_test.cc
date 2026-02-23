#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using multibody::Parser;
using systems::Context;

namespace multibody {

// MultibodyPlant friend class used to provide access to private methods for
// testing purposes.
class MultibodyPlantTester {
 public:
  static const VectorXd& EvalActuationInput(const MultibodyPlant<double>& plant,
                                            const Context<double>& context) {
    return plant.EvalActuationInput(context);
  }

  static const internal::DesiredStateInput<double>& EvalDesiredStateInput(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalDesiredStateInput(context);
  }
};

namespace {

// This fixture loads a MultibodyPlant model of a KUKA Iiiwa arm with a Schunk
// gripper.
class ActuatedIiwaArmTest : public ::testing::Test {
 public:
  // Enum to control the PD model for ther kuka arm and its gripper.
  // Unless otherwise stated, the gripper has PD control.
  // This does not affect the acrobot model, which has no PD controllers.
  enum class ModelConfiguration {
    // None of the models have PD controllers.
    kNoPdControl,
    // Arm is fully PD controlled.
    kArmIsControlled,
    // Arm has no PD control.
    kArmIsNotControlled,
    // Arm has PD control in only two of its joints.
    kArmIsPartiallyControlled,
    // Both arm and gripper have zero gains controllers.
    kModelWithZeroGains,
  };

  // Sets a model with PD controllers as specified by `model_config`. The
  // MultibodyPlant model is discrete and uses the SAP model approximation by
  // default, but this can be changed with `config`.
  void SetUpModel(
      ModelConfiguration model_config = ModelConfiguration::kArmIsNotControlled,
      const MultibodyPlantConfig& config = MultibodyPlantConfig{
          .time_step = 0.01, .discrete_contact_approximation = "sap"}) {
    const char kArmSdfUrl[] =
        "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf";
    const char kWsg50SdfUrl[] =
        "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    // Make a discrete model.
    plant_ = std::make_unique<MultibodyPlant<double>>(config.time_step);
    ApplyMultibodyPlantConfig(config, plant_.get());
    plant_->SetUseSampledOutputPorts(false);  // We're not stepping time.

    Parser parser(plant_.get());

    // Add the arm.
    arm_model_ = parser.AddModelsFromUrl(kArmSdfUrl).at(0);

    // A model of an underactuated robot.
    acrobot_model_ =
        parser
            .AddModelsFromUrl(
                "package://drake/multibody/benchmarks/acrobot/acrobot.sdf")
            .at(0);

    // Add the gripper.
    gripper_model_ = parser.AddModelsFromUrl(kWsg50SdfUrl).at(0);

    // A model of a (non-actuated) plate.
    box_model_ =
        parser.AddModelsFromUrl("package://drake/multibody/models/box.urdf")
            .at(0);

    const auto& base_body = plant_->GetBodyByName("iiwa_link_0", arm_model_);
    const auto& end_effector = plant_->GetBodyByName("iiwa_link_7", arm_model_);
    const auto& gripper_body = plant_->GetBodyByName("body", gripper_model_);
    plant_->WeldFrames(plant_->world_frame(), base_body.body_frame());
    plant_->WeldFrames(end_effector.body_frame(), gripper_body.body_frame());

    if (model_config != ModelConfiguration::kNoPdControl) {
      // Set PD controllers for the gripper.
      SetGripperModel();

      // Arm actuators.
      std::vector<JointActuatorIndex> arm_actuators;
      for (JointActuatorIndex actuator_index :
           plant_->GetJointActuatorIndices()) {
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
      } else if (model_config ==
                 ModelConfiguration::kArmIsPartiallyControlled) {
        // Add PD control only on a subset of actuators.
        auto& actuator1 = plant_->get_mutable_joint_actuator(arm_actuators[1]);
        actuator1.set_controller_gains({kProportionalGain_, kDerivativeGain_});
        auto& actuator3 = plant_->get_mutable_joint_actuator(arm_actuators[3]);
        actuator3.set_controller_gains({kProportionalGain_, kDerivativeGain_});
      } else if (model_config == ModelConfiguration::kModelWithZeroGains) {
        for (JointActuatorIndex actuator_index :
             plant_->GetJointActuatorIndices()) {
          JointActuator<double>& actuator =
              plant_->get_mutable_joint_actuator(actuator_index);
          // We do not add PD controllers to the acrobot.
          if (actuator.model_instance() == acrobot_model_) continue;
          // N.B. Proportional gains must be strictly positive, so we choose a
          // small positive number to approximate zero.
          actuator.set_controller_gains({1.0e-10, 0.0});
        }
      }
    }

    // We make the acrobot fully actuated.
    const Joint<double>& acrobot_shoulder =
        plant_->GetJointByName("ShoulderJoint", acrobot_model_);
    plant_->AddJointActuator("ShoulderJoint", acrobot_shoulder);
    // N.B. Notice that this actuator is added at a later state long after other
    // model instances were added to the plant. This will allow testing that
    // actuation input is assembled as documented by monotonically
    // increasing JointActuatorIndex, regardless of model instance index.

    if (test_remove_joint_actuators_) {
      plant_->RemoveJointActuator(
          plant_->GetJointActuatorByName("iiwa_joint_3"));
      plant_->RemoveJointActuator(plant_->GetJointActuatorByName("ElbowJoint"));
    }

    plant_->Finalize();

    context_ = plant_->CreateDefaultContext();
  }

  void SetGripperModel() {
    for (JointActuatorIndex actuator_index :
         plant_->GetJointActuatorIndices()) {
      JointActuator<double>& actuator =
          plant_->get_mutable_joint_actuator(actuator_index);
      if (actuator.model_instance() == gripper_model_) {
        actuator.set_controller_gains({kProportionalGain_, kDerivativeGain_});
      }
    }
  }

  // Makes a set of actuation values for each model instance. Values are
  // arbitrary, though non-zero.
  // If iiwa_within_limits is false, the actuation vector for the iiwa arm will
  // be above effort limits for joints 1 and 2 and below effort limits for
  // joints 4, 5, and 6. The returned tuple packs each model's actuation as
  // {arm, acrobot, gripper}.
  std::tuple<VectorXd, VectorXd, VectorXd> MakeActuationForEachModel(
      bool iiwa_within_limits, bool gripper_within_limits = true) {
    // N.B. Per SDFormat model, effort limits for the arm are
    //   [176, 176, 110, 110, 110, 40, 40] Nm.
    // Effort limits for the gripper are [80, 80] N.
    // We set some of the actuation values to be outside this limit.
    if (test_remove_joint_actuators_) {
      // iiwa_joint_3  is removed from the model.
      const VectorXd arm_u =
          iiwa_within_limits
              ? (VectorXd(6) << 50, 40, -35, -40, -35, -40).finished()
              : (VectorXd(6) << 200, 200, -160, -120, -45, -45).finished();
      const VectorXd gripper_u = gripper_within_limits
                                     ? VectorXd::LinSpaced(2, 1.0, 2.0)
                                     : Eigen::Vector2d(90, 90);
      // ElbowJoint is removed from the model.
      const Vector1d acrobot_u(3.0);
      return std::make_tuple(arm_u, acrobot_u, gripper_u);
    } else {
      const VectorXd arm_u =
          iiwa_within_limits
              ? (VectorXd(7) << 50, 40, 55, -35, -40, -35, -40).finished()
              : (VectorXd(7) << 200, 200, 55, -160, -120, -45, -45).finished();
      const VectorXd gripper_u = gripper_within_limits
                                     ? VectorXd::LinSpaced(2, 1.0, 2.0)
                                     : Eigen::Vector2d(90, 90);
      const VectorXd acrobot_u = VectorXd::LinSpaced(2, 3.0, 4.0);
      return std::make_tuple(arm_u, acrobot_u, gripper_u);
    }
  }

  // Given the actuation for each model instance separately, this function
  // assembles the actuation vector for the full MultibodyPlant model. This is
  // the actuation vector consumed by
  // MultibodyPlant::get_actuation_input_port(), ordered by JointActuatorIndex
  // (with possible gaps), regardless of model instance.
  VectorXd AssembleFullModelActuation(const VectorXd& arm_u,
                                      const VectorXd& acrobot_u,
                                      const VectorXd& gripper_u) {
    // Reported actuation values are ordered by JointActuatorIndex. That is, in
    // the order actuators were added to the model. For this test, recall that
    // the acrobot shoulder is added last programmatically.
    const int nu = plant_->num_actuated_dofs();
    if (test_remove_joint_actuators_) {
      // clang-format off
      const VectorXd full_model_u =
        (VectorXd(nu) <<
          arm_u,
          gripper_u,
          acrobot_u(0)  /* Acrobot shoulder */).finished();
      // clang-format on
      return full_model_u;
    } else {
      // clang-format off
      const VectorXd full_model_u =
        (VectorXd(nu) <<
          arm_u,
          acrobot_u(0), /* Acrobot elbow */
          gripper_u,
          acrobot_u(1)  /* Acrobot shoulder */).finished();
      // clang-format on
      return full_model_u;
    }
  }

  // This method sets arm and gripper actuation inputs with
  // MakeActuationForEachModel(false) (iiwa outside effort limits) and verifies
  // the actuation output port copies them to the output.
  // Note: Since the arm actuation is outside effort limits, for SAP this will
  // only be true in the absence of PD controllers.
  void VerifyActuationOutputFeedsThroughActuationInputs() {
    auto [arm_u, acrobot_u, gripper_u] =
        MakeActuationForEachModel(false /* iiwa outside limits */);

    // Set arbitrary actuation values.
    plant_->get_actuation_input_port(arm_model_)
        .FixValue(context_.get(), arm_u);
    plant_->get_actuation_input_port(gripper_model_)
        .FixValue(context_.get(), gripper_u);
    plant_->get_actuation_input_port(acrobot_model_)
        .FixValue(context_.get(), acrobot_u);

    // Reported actuation values are indexed by JointActuatorIndex. That is, in
    // the order actuators were added to the model. For this test, recall that
    // the acrobot elbow is added last programmatically. For continuous models,
    // we do not expect the actuators to enforce limits, see section @ref
    // mbp_actuation, in the MultibodyPlant documentation.
    const int nu = plant_->num_actuated_dofs();
    if (test_remove_joint_actuators_) {
      // clang-format off
    const VectorXd expected_u =
      (VectorXd(nu) <<
        arm_u,
        gripper_u,
        acrobot_u(1) /* Acrobot elbow */).finished();
      // clang-format on

      // Verify that net actuation output is an exact copy of the inputs.
      VerifyNetActuationOutputPorts(arm_u, acrobot_u, gripper_u, expected_u);

    } else {
      // clang-format off
    const VectorXd expected_u =
      (VectorXd(nu) <<
        arm_u,
        acrobot_u(0), /* Acrobot shoulder */
        gripper_u,
        acrobot_u(1) /* Acrobot elbow */).finished();
      // clang-format on

      // Verify that net actuation output is an exact copy of the inputs.
      VerifyNetActuationOutputPorts(arm_u, acrobot_u, gripper_u, expected_u);
    }
  }

  // Helper to verify that the net actuation output ports report the values
  // provided within `tolerance`.
  void VerifyNetActuationOutputPorts(const VectorXd& arm_u,
                                     const VectorXd& acrobot_u,
                                     const VectorXd& gripper_u,
                                     const VectorXd& u,
                                     double tolerance = 0) const {
    EXPECT_TRUE(CompareMatrices(
        plant_->get_net_actuation_output_port(arm_model_).Eval(*context_),
        arm_u, tolerance, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(
        plant_->get_net_actuation_output_port(gripper_model_).Eval(*context_),
        gripper_u, tolerance, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(
        plant_->get_net_actuation_output_port(acrobot_model_).Eval(*context_),
        acrobot_u, tolerance, MatrixCompareType::relative));
    EXPECT_TRUE(
        CompareMatrices(plant_->get_net_actuation_output_port().Eval(*context_),
                        u, tolerance, MatrixCompareType::relative));
  }

 protected:
  const int kKukaNumPositions_{7};
  const int kGripperNumPositions_{2};
  const double kProportionalGain_{10000.0};
  const double kDerivativeGain_{100.0};
  std::unique_ptr<MultibodyPlant<double>> plant_;
  ModelInstanceIndex acrobot_model_;
  ModelInstanceIndex arm_model_;
  ModelInstanceIndex gripper_model_;
  ModelInstanceIndex box_model_;
  std::unique_ptr<Context<double>> context_;
  bool test_remove_joint_actuators_{false};
};

TEST_F(ActuatedIiwaArmTest, JointActuatorApis) {
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

// We verify basic port size invariants.
TEST_F(ActuatedIiwaArmTest, GetActuationInputPort) {
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

// We verify basic port size invariants for a model in which only a subset of
// the arm's actuators are PD controlled.
TEST_F(ActuatedIiwaArmTest, GetDesiredStatePort) {
  SetUpModel(ModelConfiguration::kArmIsPartiallyControlled);

  // Whether PD-controlled or not, all desired state input ports have size
  // num_actuators(model_instance).
  EXPECT_EQ(plant_->get_desired_state_input_port(arm_model_).size(),
            2 * plant_->num_actuators(arm_model_));
  EXPECT_EQ(plant_->get_desired_state_input_port(gripper_model_).size(),
            2 * plant_->num_actuators(gripper_model_));
  EXPECT_EQ(plant_->get_desired_state_input_port(acrobot_model_).size(),
            2 * plant_->num_actuators(acrobot_model_));
  EXPECT_EQ(plant_->get_desired_state_input_port(box_model_).size(),
            2 * plant_->num_actuators(box_model_));

  // Invalid model index throws.
  const ModelInstanceIndex invalid_index(plant_->num_model_instances() + 10);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->get_desired_state_input_port(invalid_index),
      ".* get_desired_state_input_port\\(\\): condition 'model_instance < "
      "num_model_instances\\(\\)' failed.");
}

// Verify the assembly of actuation input ports. In particular, we verify this
// assembly is performed in the order of JointActuatorIndex and that
// disconnected ports default to zero values.
TEST_F(ActuatedIiwaArmTest, EvalActuationInput) {
  // We setup a model with one PD controlled model instance (the gripper) and a
  // model instance without PD control (the arm).
  SetUpModel(ModelConfiguration::kArmIsNotControlled);

  // Desired state is required to be connected, even if values are not relevant
  // for this test.
  const VectorXd gripper_xd = (VectorXd(4) << 1.0, 2.0, 3.0, 4.0).finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);

  // Fix input input ports to known values.
  // We leave the arm's port disconnected to verify it's value defaults to zero
  // actuation.
  auto [arm_u, acrobot_u, gripper_u] =
      MakeActuationForEachModel(false /* irrelevant for this test */);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_u);
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), acrobot_u);

  const VectorXd full_u =
      MultibodyPlantTester::EvalActuationInput(*plant_, *context_);

  const int arm_nu = plant_->num_actuated_dofs(arm_model_);
  VectorXd expected_u =
      AssembleFullModelActuation(VectorXd::Zero(arm_nu), acrobot_u, gripper_u);

  EXPECT_EQ(full_u, expected_u);
}

// We build an a model containing a fully PD-actuated gripper and an iiwa arm
// with only a subset of its joints PD-controlled.
TEST_F(ActuatedIiwaArmTest,
       EvalDesiredStateInput_PartiallyPdControlledModelsAreAllowed) {
  // We build an IIWA model with only a subset of actuators having PD control.
  SetUpModel(ModelConfiguration::kArmIsPartiallyControlled);

  // Desired state for the arm.
  const VectorXd arm_xd =
      VectorXd::LinSpaced(2 * kKukaNumPositions_, 1.0, 14.0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  // Verify input assembly.
  const internal::DesiredStateInput<double> input =
      MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);
  EXPECT_EQ(input.num_model_instances(), plant_->num_model_instances());
  EXPECT_TRUE(input.is_armed(arm_model_));
  EXPECT_FALSE(input.is_armed(gripper_model_));
  EXPECT_EQ(input.positions(arm_model_), arm_xd.head(kKukaNumPositions_));
  EXPECT_EQ(input.velocities(arm_model_), arm_xd.tail(kKukaNumPositions_));
}

// Verify the assembly of desired states for a plant with a single PD controlled
// model instance.
TEST_F(ActuatedIiwaArmTest, EvalDesiredStateInput_VerifyAssemblyWithOneModel) {
  SetUpModel();

  const VectorXd gripper_xd = (VectorXd(4) << 1.0, 2.0, 3.0, 4.0).finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);
  const internal::DesiredStateInput<double> input =
      MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);

  // Only the gripper model is "armed".
  EXPECT_EQ(input.num_model_instances(), plant_->num_model_instances());
  EXPECT_FALSE(input.is_armed(acrobot_model_));
  EXPECT_FALSE(input.is_armed(arm_model_));
  EXPECT_TRUE(input.is_armed(gripper_model_));
  EXPECT_FALSE(input.is_armed(box_model_));

  // Verify desired states for the only model that is armed.
  EXPECT_EQ(input.positions(gripper_model_), gripper_xd.head(2));
  EXPECT_EQ(input.velocities(gripper_model_), gripper_xd.tail(2));
}

// Verify the assembly of desired states for a plant with two PD controlled
// model instances.
TEST_F(ActuatedIiwaArmTest, EvalDesiredStateInput_VerifyAssemblyWithTwoModels) {
  SetUpModel(ModelConfiguration::kArmIsControlled);

  // Fixed desired state for the gripper.
  const VectorXd gripper_xd = (VectorXd(4) << 1.0, 2.0, 3.0, 4.0).finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);

  // We provided no desired state for the arm, and it is therefore "disarmed".
  {
    const internal::DesiredStateInput<double> input =
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);

    EXPECT_EQ(input.num_model_instances(), plant_->num_model_instances());
    EXPECT_FALSE(input.is_armed(acrobot_model_));
    EXPECT_FALSE(input.is_armed(arm_model_));
    EXPECT_TRUE(input.is_armed(gripper_model_));
    EXPECT_FALSE(input.is_armed(box_model_));
    EXPECT_EQ(input.positions(gripper_model_), gripper_xd.head(2));
    EXPECT_EQ(input.velocities(gripper_model_), gripper_xd.tail(2));
  }

  // We now do provide desired state for the arm.
  const VectorXd arm_xd = VectorXd::LinSpaced(14, 1.0, 14.0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  // We verify the arm is no longer disarmed and the input assembly.
  {
    const internal::DesiredStateInput<double> input =
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);

    // Verify desired states for the models that are armed.
    EXPECT_FALSE(input.is_armed(acrobot_model_));
    EXPECT_TRUE(input.is_armed(arm_model_));
    EXPECT_TRUE(input.is_armed(gripper_model_));
    EXPECT_FALSE(input.is_armed(box_model_));
    EXPECT_EQ(input.positions(gripper_model_), gripper_xd.head(2));
    EXPECT_EQ(input.velocities(gripper_model_), gripper_xd.tail(2));
    EXPECT_EQ(input.positions(arm_model_), arm_xd.head(7));
    EXPECT_EQ(input.velocities(arm_model_), arm_xd.tail(7));
  }
}

// Verify that an exception is thrown if there are NaNs in the desired state
// input port, only for states corresponding to PD-controlled actuators. Desired
// states for non PD-controlled actuators are ignored.
TEST_F(ActuatedIiwaArmTest, EvalDesiredStateInput_RejectNansUnlessIgnored) {
  // We build an IIWA model with only a subset of actuators having PD control.
  SetUpModel(ModelConfiguration::kArmIsPartiallyControlled);

  // Purposely inject NaN values for actuators with no PD. These should not
  // trigger an exception since they are ignored.
  {
    VectorXd arm_xd = VectorXd::LinSpaced(2 * kKukaNumPositions_, 1.0, 14.0);
    // First actuator does not have PD-control.
    const int first_actuator_index = 0;
    const JointActuator<double>& actuator = plant_->get_joint_actuator(
        plant_->GetJointActuatorIndices(arm_model_)[first_actuator_index]);
    ASSERT_FALSE(actuator.has_controller());
    arm_xd[first_actuator_index] = NAN;
    arm_xd[kKukaNumPositions_ + first_actuator_index] = NAN;
    plant_->get_desired_state_input_port(arm_model_)
        .FixValue(context_.get(), arm_xd);
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
  }

  // NaN values for PD-controlled actuators do trigger an exception, unless the
  // actuated joint is locked.
  {
    VectorXd arm_xd = VectorXd::LinSpaced(2 * kKukaNumPositions_, 1.0, 14.0);
    // Second actuator does have PD-control.
    const JointActuatorIndex pd_actuator(1);
    const auto& actuator = plant_->get_joint_actuator(pd_actuator);

    // NaN qd, valid vd.
    arm_xd[pd_actuator] = NAN;
    arm_xd[kKukaNumPositions_ + pd_actuator] = 0.0;
    plant_->get_desired_state_input_port(arm_model_)
        .FixValue(context_.get(), arm_xd);
    DRAKE_EXPECT_THROWS_MESSAGE(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_),
        "Desired state input port for model instance iiwa7 contains NaN.");
    // The NaN should be ignored if the actuated joint is locked.
    actuator.joint().Lock(context_.get());
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));

    // Valid qd, NaN vd.
    arm_xd[pd_actuator] = 0.0;
    arm_xd[kKukaNumPositions_ + pd_actuator] = NAN;
    actuator.joint().Unlock(context_.get());
    plant_->get_desired_state_input_port(arm_model_)
        .FixValue(context_.get(), arm_xd);
    DRAKE_EXPECT_THROWS_MESSAGE(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_),
        "Desired state input port for model instance iiwa7 contains NaN.");
    // The NaN should be ignored if the actuated joint is locked.
    actuator.joint().Lock(context_.get());
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
  }
}

TEST_F(ActuatedIiwaArmTest,
       PdControlledActuatorsOnlySupportedForDiscreteModels) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      SetUpModel(ModelConfiguration::kArmIsControlled,
                 MultibodyPlantConfig{.time_step = 0.0}),
      "Continuous model with PD controlled joint actuators. This feature is "
      "only supported for discrete models. Refer to MultibodyPlant's "
      "documentation for further details.");
}

// This unit test verifies that, when within effort limits, forces applied
// through the generalized forces input port has the same effect as applying the
// same forces using the actuation input port.
TEST_F(ActuatedIiwaArmTest,
       WithinEffortLimitsActuationMatchesAppliedGeneralizedForces) {
  // We add PD controllers but set their gains to zero since for this test we
  // are only interested on verifying that the effect of input actuation in the
  // dynamics is handled properly.
  SetUpModel(ModelConfiguration::kModelWithZeroGains);

  const VectorXd arm_q0 = (VectorXd(7) << 0, 0, 0, -1.7, 0, 1.0, 0).finished();
  const VectorXd arm_v0 = VectorXd::Zero(7);
  const VectorXd arm_x0 = (VectorXd(14) << arm_q0, arm_v0).finished();
  const VectorXd gripper_q0 = VectorXd::Zero(2);
  const VectorXd gripper_v0 = VectorXd::Zero(2);
  const VectorXd gripper_x0 =
      (VectorXd(4) << gripper_q0, gripper_v0).finished();

  plant_->SetPositionsAndVelocities(context_.get(), arm_model_, arm_x0);
  plant_->SetPositionsAndVelocities(context_.get(), gripper_model_, gripper_x0);

  // The desired state input ports are required to be connected, even when PD
  // gains are zero in this test.
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_x0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_x0);

  // N.B. These values of actuation are well within effort limits, for both arm
  // and gripper.
  auto [arm_u, acrobot_u, gripper_u] =
      MakeActuationForEachModel(true /* iiwa inside limits */);
  const VectorXd expected_u =
      AssembleFullModelActuation(arm_u, acrobot_u, gripper_u);

  // Map actuation to generalized forces.
  const MatrixXd B = plant_->MakeActuationMatrix();
  VectorXd tau = B * expected_u;

  auto updates = plant_->AllocateDiscreteVariables();

  // Input through generalized forces.
  plant_->get_applied_generalized_force_input_port().FixValue(context_.get(),
                                                              tau);
  plant_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());
  const VectorXd x_tau = updates->get_vector().CopyToVector();

  // Input through actuation. Zero generalized forces first.
  plant_->get_applied_generalized_force_input_port().FixValue(
      context_.get(), VectorXd::Zero(17));
  plant_->get_actuation_input_port(arm_model_).FixValue(context_.get(), arm_u);
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), acrobot_u);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_u);
  plant_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());
  const VectorXd x_actuation = updates->get_vector().CopyToVector();

  // N.B. Generalized forces inputs and actuation inputs feed into the result in
  // very different ways. Actuation input goes through the SAP solver and
  // therefore the accuracy of the solution is affected by solver tolerances.
  const double kTolerance = 1.0e-12;
  EXPECT_TRUE(CompareMatrices(x_actuation, x_tau, kTolerance,
                              MatrixCompareType::relative));

  // Verify the net actuation values reported by the plant.
  VerifyNetActuationOutputPorts(arm_u, acrobot_u, gripper_u, expected_u,
                                kTolerance);
}

// We verify that the PD controlled actuators exert effort limits.
TEST_F(ActuatedIiwaArmTest,
       OutsideEffortLimitsActuationMatchesAppliedGeneralizedForces) {
  // We add PD controllers but set their gains to zero since for this test we
  // are only interested on verifying that the effect of input actuation in the
  // dynamics is handled properly.
  SetUpModel(ModelConfiguration::kModelWithZeroGains);

  const VectorXd arm_q0 = (VectorXd(7) << 0, 0, 0, -1.7, 0, 1.0, 0).finished();
  const VectorXd arm_v0 = VectorXd::Zero(7);
  const VectorXd arm_x0 = (VectorXd(14) << arm_q0, arm_v0).finished();
  const VectorXd gripper_q0 = VectorXd::Zero(2);
  const VectorXd gripper_v0 = VectorXd::Zero(2);
  const VectorXd gripper_x0 =
      (VectorXd(4) << gripper_q0, gripper_v0).finished();

  plant_->SetPositionsAndVelocities(context_.get(), arm_model_, arm_x0);
  plant_->SetPositionsAndVelocities(context_.get(), gripper_model_, gripper_x0);

  // The desired state input ports are required to be connected, even when PD
  // gains are zero in this test.
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_x0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_x0);

  auto [arm_u, acrobot_u, gripper_u] =
      MakeActuationForEachModel(false /* iiwa outside limits */);

  // We clamp u to be within the iiwa effort limits so that we can make an
  // equivalent vector or generalized forces below.
  const VectorXd limits =
      (VectorXd(7) << 176, 176, 110, 110, 110, 40, 40).finished();
  const VectorXd arm_u_clamped =
      arm_u.array().min(limits.array()).max(-limits.array());
  const VectorXd expected_u =
      AssembleFullModelActuation(arm_u_clamped, acrobot_u, gripper_u);

  // Map actuation to generalized forces.
  const MatrixXd B = plant_->MakeActuationMatrix();
  VectorXd tau = B * expected_u;

  auto updates = plant_->AllocateDiscreteVariables();

  // Input through generalized forces.
  plant_->get_applied_generalized_force_input_port().FixValue(context_.get(),
                                                              tau);
  plant_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());
  const VectorXd x_tau = updates->get_vector().CopyToVector();

  // Input through actuation. Zero generalized forces first.
  plant_->get_applied_generalized_force_input_port().FixValue(
      context_.get(), VectorXd::Zero(17));
  plant_->get_actuation_input_port(arm_model_).FixValue(context_.get(), arm_u);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_u);
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), acrobot_u);
  plant_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());
  const VectorXd x_actuation = updates->get_vector().CopyToVector();

  // N.B. Generalized forces inputs and actuation inputs feed into the result in
  // very different ways. Actuation input goes through the SAP solver and
  // therefore the accuracy of the solution is affected by solver tolerances.
  const double kTolerance = 1.0e-10;
  EXPECT_TRUE(CompareMatrices(x_actuation, x_tau, kTolerance,
                              MatrixCompareType::relative));

  // Verify the actuation values reported by the plant.
  VerifyNetActuationOutputPorts(arm_u_clamped, acrobot_u, gripper_u, expected_u,
                                kTolerance);

  // Joint 4 is actuated beyond its effort limits. Here we verify that when the
  // joint is locked (and only for this joint), it's PD controller is ignored
  // and only the input actuation (through the actuation input port) is reported
  // in the actuation output.
  plant_->GetJointByName("iiwa_joint_4").Lock(context_.get());
  const VectorXd arm_u_when_joint4_is_locked =
      (VectorXd(7) << 176, 176, 55, -160, -110, -40, -40).finished();
  const VectorXd expected_u_when_joint4_is_locked = AssembleFullModelActuation(
      arm_u_when_joint4_is_locked, acrobot_u, gripper_u);
  VerifyNetActuationOutputPorts(arm_u_when_joint4_is_locked, acrobot_u,
                                gripper_u, expected_u_when_joint4_is_locked,
                                kTolerance);
}

// This unit test verifies that the net actuation contributions computed by SAP
// PD constraints use the correct index into the full actuation vector in the
// presence of removed actuators. Prior to having the ability to remove
// actuators, our code used the actuator's `JointActuatorIndex` as the index
// into the full actuation vector, `u`. When actuators are removed, the
// (unmodified) `JointActuatorIndex` is no longer a valid index into `u`. The
// correct offset (regardless of whether actuators have been removed or not) is
// reported by `JointActuator::input_start()`. This test in particular covers
// the indexing used in `SapDriver::CalcActuation()` which calculates the values
// reported from the net actuation output port for models using SAP with PD
// controlled actuators. Were this function to use `JointActuatorIndex`, values
// would be written to the wrong location in the output vector. We saturate the
// feed forward actuation of the gripper and arm so that the generalized forces
// reported by the constraints are clamped to the known effort limits, giving
// simple expected values.
TEST_F(ActuatedIiwaArmTest, RemovedActuatorNetActuationPDController) {
  test_remove_joint_actuators_ = true;
  // Set the PD gains to near zero to effectively disable PD control, while
  // still reporting actuation through the constraints.
  SetUpModel(ModelConfiguration::kModelWithZeroGains);

  // The desired state input ports are required to be connected, even when PD
  // gains are (effectively) zero in this test.
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), VectorXd::Zero(4));
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), VectorXd::Zero(12));

  auto [arm_u, acrobot_u, gripper_u] = MakeActuationForEachModel(
      false /* iiwa outside limits */, false /* gripper outside limits */);

  // Set the feedforward actuation input ports.
  plant_->get_actuation_input_port(arm_model_).FixValue(context_.get(), arm_u);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_u);
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), acrobot_u);

  // The feedforward actuation is well over the effort limits for the arm and
  // gripper, so we expect the PD constraints to just saturate to the effort
  // limits, thus giving us predictable values from constraint force reporting
  // (the values that eventually make their way into the net actuation output
  // ports).
  const VectorXd arm_limits =
      (VectorXd(6) << 176, 176, 110, 110, 40, 40).finished();
  const VectorXd arm_u_clamped =
      arm_u.array().min(arm_limits.array()).max(-arm_limits.array());
  // TODO(joemasterjohn): Hardcoding these limits based on the SDF model is too
  // brittle. Update this and other tests to use the actual limits found in the
  // plant.
  const VectorXd gripper_limits = Eigen::Vector2d(80, 80);
  const VectorXd gripper_u_clamped = gripper_u.array()
                                         .min(gripper_limits.array())
                                         .max(-gripper_limits.array());

  const VectorXd expected_u =
      AssembleFullModelActuation(arm_u_clamped, acrobot_u, gripper_u_clamped);

  VerifyNetActuationOutputPorts(arm_u_clamped, acrobot_u, gripper_u_clamped,
                                expected_u);
}

// This test verifies that for continuous models the actuation output port
// simply feeds through the actuation inputs.
TEST_F(ActuatedIiwaArmTest,
       ActuationOutputForContinuousModelsFeedsThroughActuationInput) {
  SetUpModel(ModelConfiguration::kNoPdControl,
             MultibodyPlantConfig{.time_step = 0.0});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

// This test verifies that discrete models using a solver other than SAP, simply
// feed through the actuation inputs.
TEST_F(ActuatedIiwaArmTest,
       ActuationOutputForDiscreteNonSapModelsFeedsThroughActuationInput) {
  SetUpModel(ModelConfiguration::kNoPdControl,
             MultibodyPlantConfig{.time_step = 0.01,
                                  .discrete_contact_approximation = "tamsi"});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

// This test verifies that SAP models without PD controllers also feed through
// the actuation input to the actuation output.
TEST_F(ActuatedIiwaArmTest,
       ActuationOutputForDiscreteSapModelsFeedsThroughActuationInput) {
  SetUpModel(ModelConfiguration::kNoPdControl,
             MultibodyPlantConfig{.time_step = 0.01,
                                  .discrete_contact_approximation = "sap"});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

// Call the same methods as the RemoveJointActuator test to confirm the values
// _without_ the actuators removed.
TEST_F(ActuatedIiwaArmTest, DontRemoveJointActuator) {
  SetUpModel();

  EXPECT_EQ(plant_->num_actuators(), 7 + 2 + 2);
  EXPECT_EQ(plant_->num_actuators(arm_model_), 7);
  EXPECT_EQ(plant_->num_actuators(acrobot_model_), 2);
  EXPECT_EQ(plant_->num_actuators(gripper_model_), 2);

  EXPECT_EQ(plant_->num_actuated_dofs(), 7 + 2 + 2);
  EXPECT_EQ(plant_->num_actuated_dofs(arm_model_), 7);
  EXPECT_EQ(plant_->num_actuated_dofs(acrobot_model_), 2);
  EXPECT_EQ(plant_->num_actuated_dofs(gripper_model_), 2);

  EXPECT_TRUE(plant_->HasJointActuatorNamed("iiwa_joint_3"));
  EXPECT_TRUE(plant_->HasJointActuatorNamed("iiwa_joint_3", arm_model_));
  EXPECT_TRUE(plant_->HasJointActuatorNamed("ElbowJoint"));
  EXPECT_TRUE(plant_->HasJointActuatorNamed("ElbowJoint", acrobot_model_));

  const JointActuator<double>& iiwa_joint_3_actuator =
      plant_->GetJointActuatorByName("iiwa_joint_3");
  EXPECT_EQ(plant_->GetJointActuatorByName("iiwa_joint_3", arm_model_).index(),
            iiwa_joint_3_actuator.index());
  EXPECT_TRUE(plant_->has_joint_actuator(iiwa_joint_3_actuator.index()));

  // GetJointActuatorIndices.
  EXPECT_EQ(plant_->GetJointActuatorIndices().size(), 7 + 2 + 2);
  const JointActuator<double>& shoulder_joint_actuator =
      plant_->GetJointActuatorByName("ShoulderJoint");
  const JointActuator<double>& elbow_joint_actuator =
      plant_->GetJointActuatorByName("ElbowJoint");
  EXPECT_THAT(plant_->GetJointActuatorIndices(acrobot_model_),
              testing::ElementsAre(elbow_joint_actuator.index(),
                                   shoulder_joint_actuator.index()));
}

TEST_F(ActuatedIiwaArmTest, RemoveJointActuator) {
  // Remove iiwa_joint_3 and ElbowJoint from the model.
  test_remove_joint_actuators_ = true;
  SetUpModel();

  EXPECT_EQ(plant_->num_actuators(), 6 + 1 + 2);
  EXPECT_EQ(plant_->num_actuators(arm_model_), 6);
  EXPECT_EQ(plant_->num_actuators(acrobot_model_), 1);
  EXPECT_EQ(plant_->num_actuators(gripper_model_), 2);

  EXPECT_EQ(plant_->num_actuated_dofs(), 6 + 1 + 2);
  EXPECT_EQ(plant_->num_actuated_dofs(arm_model_), 6);
  EXPECT_EQ(plant_->num_actuated_dofs(acrobot_model_), 1);
  EXPECT_EQ(plant_->num_actuated_dofs(gripper_model_), 2);

  EXPECT_FALSE(plant_->HasJointActuatorNamed("iiwa_joint_3"));
  EXPECT_FALSE(plant_->HasJointActuatorNamed("iiwa_joint_3", arm_model_));
  EXPECT_FALSE(plant_->HasJointActuatorNamed("ElbowJoint"));
  EXPECT_FALSE(plant_->HasJointActuatorNamed("ElbowJoint", acrobot_model_));

  // GetJointActuatorIndices.
  EXPECT_EQ(plant_->GetJointActuatorIndices().size(), 6 + 1 + 2);
  const JointActuator<double>& shoulder_joint_actuator =
      plant_->GetJointActuatorByName("ShoulderJoint");
  EXPECT_THAT(plant_->GetJointActuatorIndices(acrobot_model_),
              testing::ElementsAre(shoulder_joint_actuator.index()));
}

GTEST_TEST(ActuatedModelsTest, ZeroActuationPriorToStepping) {
  // Load a discrete model.
  const char kArmSdfUrl[] =
      "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf";
  auto plant = std::make_unique<MultibodyPlant<double>>(0.01);
  Parser parser(plant.get());
  parser.AddModelsFromUrl(kArmSdfUrl);
  plant->Finalize();

  // Provide actuation input.
  auto model = plant->GetModelInstanceByName("iiwa7");
  auto context = plant->CreateDefaultContext();
  plant->get_actuation_input_port(model).FixValue(
      context.get(), Eigen::VectorXd::Constant(7, 1.0));

  // Evaluate all of the actuation output ports to confirm they are zero.
  EXPECT_TRUE(plant->get_net_actuation_output_port().Eval(*context).isZero());
  for (ModelInstanceIndex i{0}; i < plant->num_model_instances(); ++i) {
    const auto& ith_port = plant->get_net_actuation_output_port(i);
    EXPECT_TRUE(ith_port.Eval(*context).isZero());
  }

  // Take a step and now they are non-zero.
  plant->ExecuteForcedEvents(context.get());
  EXPECT_FALSE(plant->get_net_actuation_output_port().Eval(*context).isZero());
  const auto& model_port = plant->get_net_actuation_output_port(model);
  EXPECT_FALSE(model_port.Eval(*context).isZero());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
