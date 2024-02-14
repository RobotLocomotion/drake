#include <limits>
#include <memory>
#include <tuple>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
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
  // Enum to control the PD model for ther kuka arm and its gripper.
  // This does not affect the acrobot model, which has no PD controllers.
  enum class ModelConfiguration {
    // None of the models have PD controllers.
    kNoPdControl,
    kArmIsControlled,
    kArmIsNotControlled,
    kArmIsPartiallyControlled,
    kModelWithZeroGains,
  };

  // Sets a model with PD controllers as specified by `model_config`. The
  // MultibodyPlant model is discrete and uses the SAP model approximation by
  // default, but this can be changed with `config`.
  void SetUpModel(
      ModelConfiguration model_config = ModelConfiguration::kArmIsNotControlled,
      const MultibodyPlantConfig& config = MultibodyPlantConfig{
          .time_step = 0.01, .discrete_contact_approximation = "sap"}) {
    const char kArmSdfPath[] =
        "drake/manipulation/models/iiwa_description/iiwa7/"
        "iiwa7_no_collision.sdf";

    const char kWsg50SdfPath[] =
        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    // Make a discrete model.
    plant_ = std::make_unique<MultibodyPlant<double>>(config.time_step);
    ApplyMultibodyPlantConfig(config, plant_.get());

    Parser parser(plant_.get());

    // Add the arm.
    arm_model_ = parser.AddModels(FindResourceOrThrow(kArmSdfPath)).at(0);

    // A model of an underactuated robot.
    acrobot_model_ = parser
                         .AddModels(FindResourceOrThrow(
                             "drake/multibody/benchmarks/acrobot/acrobot.sdf"))
                         .at(0);

    // Add the gripper.
    gripper_model_ = parser.AddModels(FindResourceOrThrow(kWsg50SdfPath)).at(0);

    // A model of a (non-actuated) plate.
    box_model_ =
        parser.AddModels(FindResourceOrThrow("drake/multibody/models/box.urdf"))
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
      } else if (model_config ==
                 ModelConfiguration::kArmIsPartiallyControlled) {
        // Add PD control only on a subset of actuators.
        auto& actuator1 = plant_->get_mutable_joint_actuator(arm_actuators[1]);
        actuator1.set_controller_gains({kProportionalGain_, kDerivativeGain_});
        auto& actuator3 = plant_->get_mutable_joint_actuator(arm_actuators[3]);
        actuator3.set_controller_gains({kProportionalGain_, kDerivativeGain_});
      } else if (model_config == ModelConfiguration::kModelWithZeroGains) {
        for (JointActuatorIndex actuator_index(0);
             actuator_index < plant_->num_actuators(); ++actuator_index) {
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
    plant_->AddJointActuator("ShoulderActuator", acrobot_shoulder);
    // N.B. Notice that this actuator is added at a later state long after other
    // model instances were added to the plant. This will allow testing that
    // actuation input is assembled as documented by monotonically
    // increasing JointActuatorIndex, regardless of model instance index.

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

  // Makes a set of actuation values for each model instance. Values are
  // arbitrary, though non-zero.
  // If iiwa_within_limits is false, the actuation vector for the iiwa arm will
  // be above effort limits for joints 1 and 2 and below effort limits for
  // joints 4, 5, and 6. The returned tuple packs each model's actuation as
  // {arm, acrobot, gripper, box}.
  static std::tuple<VectorXd, VectorXd, VectorXd> MakeActuationForEachModel(
      bool iiwa_within_limits) {
    // N.B. Per SDFormat model, effort limits are [176, 176, 110, 110, 110,
    // 40, 40] Nm.
    // We set some of the actuation values to be outside this limit.
    const VectorXd arm_u =
        iiwa_within_limits
            ? (VectorXd(7) << 50, 40, 55, -35, -40, -35, -40).finished()
            : (VectorXd(7) << 200, 200, 55, -160, -120, -45, -45).finished();
    const VectorXd gripper_u = VectorXd::LinSpaced(2, 1.0, 2.0);
    const VectorXd acrobot_u = VectorXd::LinSpaced(2, 3.0, 4.0);
    return std::make_tuple(arm_u, acrobot_u, gripper_u);
  }

  // Given the actuation for each model instance separately, this function
  // assembles the actuation vector for the full MultibodyPlant model. This is
  // the actuation vector consumed by
  // MultibodyPlant::get_actuation_input_port(), ordered by JointActuatorIndex,
  // regardless of model instance.
  VectorXd AssembleFullModelActuation(const VectorXd& arm_u,
                                      const VectorXd& acrobot_u,
                                      const VectorXd& gripper_u) {
    // Reported actuation values are indexed by JointActuatorIndex. That is, in
    // the order actuators were added to the model. For this test, recall that
    // the acrobot shoulder is added last programmatically.
    const int nu = plant_->num_actuated_dofs();
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

// Verify the assembly of actuation input ports. In particular, we verify this
// assembly is performed in the order of JointActuatorIndex and that
// disconnected ports default to zero values.
TEST_F(ActuatedIiiwaArmTest, AssembleActuationInput) {
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
      MultibodyPlantTester::AssembleActuationInput(*plant_, *context_);

  const int arm_nu = plant_->num_actuated_dofs(arm_model_);
  VectorXd expected_u =
      AssembleFullModelActuation(VectorXd::Zero(arm_nu), acrobot_u, gripper_u);

  EXPECT_EQ(full_u, expected_u);
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
      "Model iiwa7 is partially PD controlled. .*");
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

  const VectorXd gripper_xd = (VectorXd(4) << 1.0, 2.0, 3.0, 4.0).finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);
  const VectorXd full_xd =
      MultibodyPlantTester::AssembleDesiredStateInput(*plant_, *context_);

  const int nu = plant_->num_actuated_dofs();
  // AssembleDesiredStateInput will always return a vector of size 2 * nu. It
  // fills in values for those models with PD control and set all other entries
  // to zero. In this case, entries corresponding to the arm are expected to be
  // zero. All qd values go first, followed by vd values.
  const int arm_nu = plant_->num_actuated_dofs(arm_model_);
  // clang-format off
  VectorXd expected_xd =
      (VectorXd(2 * nu) <<
        // Desired positions.
        VectorXd::Zero(arm_nu),
        0.0, /* Acrobot shoulder */
        gripper_xd.head<2>(),
        0.0, /* Acrobot elbow */
        // Desired velocities.
        VectorXd::Zero(arm_nu),
        0.0, /* Acrobot shoulder */
        gripper_xd.tail<2>(),
        0.0 /* Acrobot elbow */).finished();
  // clang-format on

  EXPECT_EQ(full_xd, expected_xd);
}

// Verify the assembly of desired states for a plant with two PD controlled
// model instances.
TEST_F(ActuatedIiiwaArmTest,
       AssembleDesiredStateInput_VerifyAssemblyWithTwoModels) {
  SetUpModel(ModelConfiguration::kArmIsControlled);

  // Fixed desired state input ports to known values.
  // Both arm and gripper are required to be connected.
  const VectorXd gripper_xd = (VectorXd(4) << 1.0, 2.0, 3.0, 4.0).finished();
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
  // clang-format off
  const VectorXd expected_xd =
      (VectorXd(2 * nu) <<
        // Desired positions.
        arm_xd.head<7>(),
        0.0, /* Acrobot shoulder */
        gripper_xd.head<2>(),
        0.0, /* Acrobot elbow */
        // Desired velocities.
        arm_xd.tail<7>(),
        0.0, /* Acrobot shoulder */
        gripper_xd.tail<2>(),
        0.0 /* Acrobot elbow */).finished();
  // clang-format on

  EXPECT_EQ(full_xd, expected_xd);
}

TEST_F(ActuatedIiiwaArmTest,
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
TEST_F(ActuatedIiiwaArmTest,
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
TEST_F(ActuatedIiiwaArmTest,
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

// This test verifies that for continuous models the actuation output port
// simply feeds through the actuation inputs.
TEST_F(ActuatedIiiwaArmTest,
       ActuationOutputForContinuousModelsFeedsThroughActuationInput) {
  SetUpModel(ModelConfiguration::kNoPdControl,
             MultibodyPlantConfig{.time_step = 0.0});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

// This test verifies that discrete models using a solver other than SAP, simply
// feed through the actuation inputs.
TEST_F(ActuatedIiiwaArmTest,
       ActuationOutputForDiscreteNonSapModelsFeedsThroughActuationInput) {
  SetUpModel(ModelConfiguration::kNoPdControl,
             MultibodyPlantConfig{.time_step = 0.01,
                                  .discrete_contact_approximation = "tamsi"});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

// This test verifies that SAP models without PD controllers also feed through
// the actuation input to the actuation output.
TEST_F(ActuatedIiiwaArmTest,
       ActuationOutputForDiscreteSapModelsFeedsThroughActuationInput) {
  SetUpModel(ModelConfiguration::kNoPdControl,
             MultibodyPlantConfig{.time_step = 0.01,
                                  .discrete_contact_approximation = "sap"});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

}  // namespace
}  // namespace multibody
}  // namespace drake
