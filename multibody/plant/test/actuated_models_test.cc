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
#include "drake/systems/framework/context.h"

using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {

// MultibodyPlant friend class used to provide access to private methods for
// testing purposes.
class MultibodyPlantTester {
 public:
  static const VectorXd& EvalActuationInput(const MultibodyPlant<double>& plant,
                                            const Context<double>& context,
                                            bool effort_limit) {
    return plant.EvalActuationInput(context, effort_limit);
  }

  static const internal::DesiredStateInput<double>& EvalDesiredStateInput(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalDesiredStateInput(context);
  }
};

namespace {

struct TestParam {
  bool remove_joint_actuators{false};
  // - discrete time unsampled vs discrete time sampled (vs continuous time)
  // - actuation input over limit yes/no
  // - desired state is connected yes/no
  // - model is fully pd controlled vs partially
  // - joint is locked
};

// This fixture loads a MultibodyPlant with:
// - a KUKA Iiwa arm with a Schunk WSG gripper (nominally both fully actuated),
// - an acrobot (nominally fully actuated), and
// - a floating box (un-actuated).
//
// If `remove_joint_actuators` is set, one joint each from the Iiwa and acrobot
// will be unactuated.
class ActuatedModelsTest : public ::testing::TestWithParam<TestParam> {
 public:
  ActuatedModelsTest() : param_(GetParam()) {}

  // By default, the MultibodyPlant model is discrete and uses the SAP model
  // approximation, but this can be changed with `config`.
  void SetUpModel(const MultibodyPlantConfig& config =
                      MultibodyPlantConfig{
                          .time_step = 0.01,
                          .discrete_contact_approximation = "sap"},
                  const bool finalize = true) {
    // Construct the plant in the requested mode.
    plant_ = std::make_unique<MultibodyPlant<double>>(config.time_step);
    ApplyMultibodyPlantConfig(config, plant_.get());
    plant_->SetUseSampledOutputPorts(false);  // We're not stepping time.

    // Add the models.
    Parser parser(plant_.get());
    constexpr char arm_url[] =
        "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf";
    constexpr char acrobot_url[] =
        "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
    constexpr char gripper_url[] =
        "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf";
    constexpr char box_url[] = "package://drake/multibody/models/box.urdf";
    arm_model_ = parser.AddModelsFromUrl(arm_url).at(0);
    acrobot_model_ = parser.AddModelsFromUrl(acrobot_url).at(0);
    gripper_model_ = parser.AddModelsFromUrl(gripper_url).at(0);
    box_model_ = parser.AddModelsFromUrl(box_url).at(0);
    const auto& iiwa_link_0 = plant_->GetBodyByName("iiwa_link_0", arm_model_);
    const auto& iiwa_link_7 = plant_->GetBodyByName("iiwa_link_7", arm_model_);
    const auto& gripper_body = plant_->GetBodyByName("body", gripper_model_);
    plant_->WeldFrames(plant_->world_frame(), iiwa_link_0.body_frame());
    plant_->WeldFrames(iiwa_link_7.body_frame(), gripper_body.body_frame());

    // Make the acrobot fully actuated. Notice that this actuator is added after
    // other model instances. This will allow testing that actuation input is
    // assembled as documented by monotonically increasing JointActuatorIndex,
    // regardless of ModelInstanceIndex.
    const Joint<double>& acrobot_shoulder =
        plant_->GetJointByName("ShoulderJoint", acrobot_model_);
    plant_->AddJointActuator("ShoulderJoint", acrobot_shoulder);

    // Optionally change the iiwa and acrobot to underactuated.
    if (param_.remove_joint_actuators) {
      for (const auto& actuator_name : {"iiwa_joint_3", "ElbowJoint"}) {
        plant_->RemoveJointActuator(
            plant_->GetJointActuatorByName(actuator_name));
      }
    }

    // Finalize.
    if (finalize) {
      plant_->Finalize();
      context_ = plant_->CreateDefaultContext();
    }
  }

  void SetControllerGains(const ModelInstanceIndex model_instance,
                          const PdControllerGains& gains = {kProportionalGain,
                                                            kDerivativeGain},
                          bool skip_every_other = false) {
    // SetUpModel() must happen first.
    DRAKE_DEMAND(plant_ != nullptr);
    DRAKE_DEMAND(plant_->is_finalized());
    int count = 0;
    for (const auto& i : plant_->GetJointActuatorIndices(model_instance)) {
      auto& actuator = plant_->get_mutable_joint_actuator(i);
      if (skip_every_other && (count % 2 == 0)) {
        // When requested, we skip the 0th, 2nd, 4th, etc. actuators within this
        // model instance.
      } else {
        actuator.set_controller_gains(gains);
      }
      ++count;
    }
  }

  int expected_num_actuators(const ModelInstanceIndex model_instance) const {
    if (model_instance == arm_model_) {
      return param_.remove_joint_actuators ? 6 : 7;
    }
    if (model_instance == acrobot_model_) {
      return param_.remove_joint_actuators ? 1 : 2;
    }
    if (model_instance == gripper_model_) {
      return 2;
    }
    if (model_instance == box_model_) {
      return 0;
    }
    DRAKE_UNREACHABLE();
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
    if (param_.remove_joint_actuators) {
      // iiwa_joint_3 is removed from the model.
      const VectorXd arm_u =
          iiwa_within_limits
              ? (VectorXd(6) << 50, 40, -35, -40, -35, -40).finished()
              : (VectorXd(6) << 200, 200, -160, -120, -45, -45).finished();
      const VectorXd gripper_u = gripper_within_limits
                                     ? VectorXd::LinSpaced(2, 1.0, 2.0)
                                     : Vector2d(90, 90);
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
                                     : Vector2d(90, 90);
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
    if (param_.remove_joint_actuators) {
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

 protected:
  static constexpr int kKukaNumPositions{7};
  static constexpr double kProportionalGain{10000.0};
  static constexpr double kDerivativeGain{100.0};

  const TestParam param_;

  std::unique_ptr<MultibodyPlant<double>> plant_;
  ModelInstanceIndex arm_model_;
  ModelInstanceIndex acrobot_model_;
  ModelInstanceIndex gripper_model_;
  ModelInstanceIndex box_model_;
  std::unique_ptr<Context<double>> context_;

  // };
  // class ActuatedModelsTest : public ActuatedModelsTestFixture {};
  // class FooTest : public ActuatedModelsTestFixture {

  // This method sets arm and gripper actuation inputs with
  // MakeActuationForEachModel(true) (iiwa inside effort limits) and verifies
  // the actuation output port copies them to the output.
  void VerifyActuationOutputFeedsThroughActuationInputs() {
    auto [arm_u, acrobot_u, gripper_u] =
        MakeActuationForEachModel(true /* iiwa inside limits */);

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
    if (param_.remove_joint_actuators) {
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
};

// Check setting and clearing PD gains.
TEST_P(ActuatedModelsTest, GetSetPdGains) {
  SetUpModel();

  SetControllerGains(gripper_model_);
  for (const auto& i : plant_->GetJointActuatorIndices(gripper_model_)) {
    const auto& actuator = plant_->get_joint_actuator(i);
    ASSERT_TRUE(actuator.has_controller());
    const PdControllerGains& gains = actuator.get_controller_gains();
    EXPECT_EQ(gains.p, kProportionalGain);
    EXPECT_EQ(gains.d, kDerivativeGain);
  }

  SetControllerGains(gripper_model_, {0.0, 0.0});
  for (const auto& i : plant_->GetJointActuatorIndices(gripper_model_)) {
    EXPECT_FALSE(plant_->get_joint_actuator(i).has_controller());
  }
}

// Check input ports and their sizes. All model instances have an actuation
// input port and desired state input port, even for unactuated models. The
// presence or lack of PD control does not affect the port size.
TEST_P(ActuatedModelsTest, InputPortSize) {
  SetUpModel();
  SetControllerGains(gripper_model_);

  int num_actuators_total = 0;
  for (const auto& model :
       {arm_model_, acrobot_model_, gripper_model_, box_model_}) {
    const int expected = expected_num_actuators(model);
    EXPECT_EQ(plant_->num_actuated_dofs(model), expected);
    EXPECT_EQ(plant_->num_actuators(model), expected);
    EXPECT_EQ(plant_->get_actuation_input_port(model).size(), expected);
    EXPECT_EQ(plant_->get_desired_state_input_port(model).size(), 2 * expected);
    num_actuators_total += expected;
  }
  EXPECT_EQ(plant_->num_actuated_dofs(), num_actuators_total);
  EXPECT_EQ(plant_->num_actuators(), num_actuators_total);
  EXPECT_EQ(plant_->get_actuation_input_port().size(), num_actuators_total);
}

// Accessing an invalid model instance's input port will throw.
TEST_P(ActuatedModelsTest, InputPortInvalid) {
  SetUpModel();
  const ModelInstanceIndex invalid_index(10);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->get_actuation_input_port(invalid_index),
      ".*get_actuation_input_port.*num_model_instances.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->get_desired_state_input_port(invalid_index),
      ".*get_desired_state_input_port.*num_model_instances.*");
}

// Verify the assembly of actuation input ports. In particular, we verify this
// assembly is performed in the order of JointActuatorIndex and disconnected
// ports default to zero values.
TEST_P(ActuatedModelsTest, EvalActuationInput) {
  SetUpModel();

  // Adding PD control should be harmless (doesn't affect actuation input).
  SetControllerGains(gripper_model_);

  // Arbitrary expected actuation.
  const VectorXd arm_u = VectorXd::Zero(expected_num_actuators(arm_model_));
  const VectorXd acrobot_u =
      VectorXd::LinSpaced(expected_num_actuators(acrobot_model_), 1.0, 2.0);
  const VectorXd gripper_u =
      VectorXd::LinSpaced(expected_num_actuators(gripper_model_), 3.0, 4.0);

  // We leave the arm's port disconnected to verify its value defaults to zero.
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), acrobot_u);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_u);

  // Check the input port assembly.
  const VectorXd full_u = MultibodyPlantTester::EvalActuationInput(
      *plant_, *context_, /* effort_limit = */ false);
  const VectorXd expected_u =
      AssembleFullModelActuation(arm_u, acrobot_u, gripper_u);
  EXPECT_EQ(full_u, expected_u);
}

// We build a model containing a fully PD-actuated gripper and an iiwa arm
// with only a subset of its joints PD-controlled.
TEST_P(ActuatedModelsTest,
       EvalDesiredStateInput_PartiallyPdControlledModelsAreAllowed) {
  // We build an IIWA model with only a subset of actuators having PD control.
  SetUpModel();
  SetControllerGains(arm_model_, {kProportionalGain, kDerivativeGain},
                     /* skip_every_other = */ true);
  SetControllerGains(gripper_model_);

  // Desired state for the arm.
  const int num_actuators = expected_num_actuators(arm_model_);
  const VectorXd arm_xd = VectorXd::LinSpaced(2 * num_actuators, 1.0, 2.0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  // Verify input assembly.
  const internal::DesiredStateInput<double> input =
      MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);
  EXPECT_EQ(input.num_model_instances(), plant_->num_model_instances());
  EXPECT_TRUE(input.is_armed(arm_model_));
  EXPECT_FALSE(input.is_armed(gripper_model_));
  EXPECT_EQ(input.positions(arm_model_), arm_xd.head(num_actuators));
  EXPECT_EQ(input.velocities(arm_model_), arm_xd.tail(num_actuators));
}

// Verify the assembly of desired states for a plant with a single PD controlled
// model instance.
TEST_P(ActuatedModelsTest, EvalDesiredStateInput_VerifyAssemblyWithOneModel) {
  SetUpModel();
  SetControllerGains(gripper_model_);

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
TEST_P(ActuatedModelsTest, EvalDesiredStateInput_VerifyAssemblyWithTwoModels) {
  SetUpModel();
  SetControllerGains(arm_model_);
  SetControllerGains(gripper_model_);

  // Fixed desired state for the gripper.
  const VectorXd gripper_xd =
      VectorXd::LinSpaced(2 * expected_num_actuators(gripper_model_), 1.0, 2.0);
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);

  // We provided no desired state for the arm, and it is therefore "disarmed".
  // Only the gripper is armed.
  {
    const internal::DesiredStateInput<double> input =
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);
    EXPECT_EQ(input.num_model_instances(), plant_->num_model_instances());
    EXPECT_FALSE(input.is_armed(acrobot_model_));
    EXPECT_FALSE(input.is_armed(arm_model_));
    EXPECT_TRUE(input.is_armed(gripper_model_));
    EXPECT_FALSE(input.is_armed(box_model_));
    EXPECT_EQ(input.positions(gripper_model_),
              gripper_xd.head(gripper_xd.size() / 2));
    EXPECT_EQ(input.velocities(gripper_model_),
              gripper_xd.tail(gripper_xd.size() / 2));
  }

  // We now do provide desired state for the arm.
  const VectorXd arm_xd =
      VectorXd::LinSpaced(2 * expected_num_actuators(arm_model_), 3.0, 4.0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  // Now both the arm and gripper are "armed".
  {
    const internal::DesiredStateInput<double> input =
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);
    EXPECT_EQ(input.num_model_instances(), plant_->num_model_instances());
    EXPECT_FALSE(input.is_armed(acrobot_model_));
    EXPECT_TRUE(input.is_armed(arm_model_));
    EXPECT_TRUE(input.is_armed(gripper_model_));
    EXPECT_FALSE(input.is_armed(box_model_));
    EXPECT_EQ(input.positions(gripper_model_),
              gripper_xd.head(gripper_xd.size() / 2));
    EXPECT_EQ(input.velocities(gripper_model_),
              gripper_xd.tail(gripper_xd.size() / 2));
    EXPECT_EQ(input.positions(arm_model_), arm_xd.head(arm_xd.size() / 2));
    EXPECT_EQ(input.velocities(arm_model_), arm_xd.tail(arm_xd.size() / 2));
  }
}

// Verify that an exception is thrown if there are NaNs in the desired state
// input port, only for states corresponding to PD-controlled actuators. Desired
// states for non PD-controlled actuators are ignored.
TEST_P(ActuatedModelsTest, EvalDesiredStateInput_RejectNansUnlessIgnored) {
  if (param_.remove_joint_actuators) {
    // XXX Fix this to pass either way.
    return;
  }
  // We build an IIWA model with only a subset of actuators having PD control.
  SetUpModel();
  SetControllerGains(arm_model_, {kProportionalGain, kDerivativeGain},
                     /* skip_every_other = */ true);
  SetControllerGains(gripper_model_);

  // Purposely inject NaN values for actuators with no PD. These should not
  // trigger an exception since they are ignored.
  {
    VectorXd arm_xd = VectorXd::LinSpaced(2 * kKukaNumPositions, 1.0, 14.0);
    // First actuator does not have PD-control.
    const int first_actuator_index = 0;
    const JointActuator<double>& actuator = plant_->get_joint_actuator(
        plant_->GetJointActuatorIndices(arm_model_)[first_actuator_index]);
    ASSERT_FALSE(actuator.has_controller());
    arm_xd[first_actuator_index] = NAN;
    arm_xd[kKukaNumPositions + first_actuator_index] = NAN;
    plant_->get_desired_state_input_port(arm_model_)
        .FixValue(context_.get(), arm_xd);
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
  }

  // NaN values for PD-controlled actuators do trigger an exception, unless the
  // actuated joint is locked.
  {
    VectorXd arm_xd = VectorXd::LinSpaced(2 * kKukaNumPositions, 1.0, 14.0);
    // Second actuator does have PD-control.
    const JointActuatorIndex pd_actuator(1);
    const auto& actuator = plant_->get_joint_actuator(pd_actuator);
    ASSERT_TRUE(actuator.has_controller());

    // NaN qd, valid vd.
    arm_xd[pd_actuator] = NAN;
    arm_xd[kKukaNumPositions + pd_actuator] = 0.0;
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
    arm_xd[kKukaNumPositions + pd_actuator] = NAN;
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

TEST_P(ActuatedModelsTest,
       PdControlledActuatorsOnlySupportedForDiscreteModels) {
  SetUpModel(MultibodyPlantConfig{.time_step = 0.0}, /* finalize = */ false);
  plant_->get_mutable_joint_actuator(JointActuatorIndex{0})
      .set_controller_gains({.p = 1.0});
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->Finalize(),
      "Continuous model with PD controlled joint actuators. This feature is "
      "only supported for discrete models. Refer to MultibodyPlant's "
      "documentation for further details.");
}

// This unit test verifies that, when within effort limits, forces applied
// through the generalized forces input port has the same effect as applying the
// same forces using the actuation input port.
TEST_P(ActuatedModelsTest,
       WithinEffortLimitsActuationMatchesAppliedGeneralizedForces) {
  if (param_.remove_joint_actuators) {
    // XXX Fix this to pass either way.
    return;
  }
  // We add PD controllers but set their gains to near-zero since for this test
  // we are only interested on verifying that the effect of input actuation in
  // the dynamics is handled properly.
  SetUpModel();
  SetControllerGains(arm_model_, {1.0e-10, 0.0});
  SetControllerGains(gripper_model_, {1.0e-10, 0.0});

  const VectorXd arm_q0 = (VectorXd(7) << 0, 0, 0, -1.7, 0, 1.0, 0).finished();
  const VectorXd arm_v0 = VectorXd::Zero(7);
  const VectorXd arm_x0 = (VectorXd(14) << arm_q0, arm_v0).finished();
  const VectorXd gripper_q0 = VectorXd::Zero(2);
  const VectorXd gripper_v0 = VectorXd::Zero(2);
  const VectorXd gripper_x0 =
      (VectorXd(4) << gripper_q0, gripper_v0).finished();

  plant_->SetPositionsAndVelocities(context_.get(), arm_model_, arm_x0);
  plant_->SetPositionsAndVelocities(context_.get(), gripper_model_, gripper_x0);

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
TEST_P(ActuatedModelsTest,
       OutsideEffortLimitsActuationMatchesAppliedGeneralizedForces) {
  if (param_.remove_joint_actuators) {
    // XXX Fix this to pass either way.
    return;
  }
  // We add PD controllers but set their gains to near-zero since for this test
  // we are only interested on verifying that the effect of input actuation in
  // the dynamics is handled properly.
  SetUpModel();
  SetControllerGains(arm_model_, {1.0e-10, 0.0});
  SetControllerGains(gripper_model_, {1.0e-10, 0.0});

  const VectorXd arm_q0 = (VectorXd(7) << 0, 0, 0, -1.7, 0, 1.0, 0).finished();
  const VectorXd arm_v0 = VectorXd::Zero(7);
  const VectorXd arm_x0 = (VectorXd(14) << arm_q0, arm_v0).finished();
  const VectorXd gripper_q0 = VectorXd::Zero(2);
  const VectorXd gripper_v0 = VectorXd::Zero(2);
  const VectorXd gripper_x0 =
      (VectorXd(4) << gripper_q0, gripper_v0).finished();

  plant_->SetPositionsAndVelocities(context_.get(), arm_model_, arm_x0);
  plant_->SetPositionsAndVelocities(context_.get(), gripper_model_, gripper_x0);

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
  {
    SCOPED_TRACE(fmt::format("net actuation, nominal"));
    VerifyNetActuationOutputPorts(arm_u_clamped, acrobot_u, gripper_u,
                                  expected_u, kTolerance);
  }

  // Locking a joint disables the (non-existent) PD controller, but otherwise
  // doesn't change anything. In particular, the effort limit is still enforced.
  {
    SCOPED_TRACE(fmt::format("net actuation, locked"));
    plant_->GetJointByName("iiwa_joint_4").Lock(context_.get());
    VerifyNetActuationOutputPorts(arm_u_clamped, acrobot_u, gripper_u,
                                  expected_u, kTolerance);
  }
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
TEST_P(ActuatedModelsTest, RemovedActuatorNetActuationPDController) {
  if (!param_.remove_joint_actuators) {
    // Skip it. We specifically only want to test with a removed actuator.
    return;
  }
  // Set the PD gains to near-zero to effectively disable PD control, while
  // still reporting actuation through the constraints.
  SetUpModel();
  SetControllerGains(arm_model_, {1.0e-10, 0.0});
  SetControllerGains(gripper_model_, {1.0e-10, 0.0});

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
  const VectorXd gripper_limits = Vector2d(80, 80);
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
TEST_P(ActuatedModelsTest,
       ActuationOutputForContinuousModelsFeedsThroughActuationInput) {
  if (param_.remove_joint_actuators) {
    // XXX Fix this to pass either way.
    return;
  }
  SetUpModel(MultibodyPlantConfig{.time_step = 0.0});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

// This test verifies that discrete models using a solver other than SAP, simply
// feed through the actuation inputs.
TEST_P(ActuatedModelsTest,
       ActuationOutputForDiscreteNonSapModelsFeedsThroughActuationInput) {
  if (param_.remove_joint_actuators) {
    // XXX Fix this to pass either way.
    return;
  }
  SetUpModel(MultibodyPlantConfig{.time_step = 0.01,
                                  .discrete_contact_approximation = "tamsi"});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

// This test verifies that SAP models without PD controllers also feed through
// the actuation input to the actuation output.
TEST_P(ActuatedModelsTest,
       ActuationOutputForDiscreteSapModelsFeedsThroughActuationInput) {
  if (param_.remove_joint_actuators) {
    // XXX Fix this to pass either way.
    return;
  }
  SetUpModel(MultibodyPlantConfig{.time_step = 0.01,
                                  .discrete_contact_approximation = "sap"});
  VerifyActuationOutputFeedsThroughActuationInputs();
}

TEST_P(ActuatedModelsTest, RemoveJointActuator) {
  SetUpModel();

  // Check whether iiwa_joint_3 was removed from the Iiwa model:
  // - HasJointActuatorNamed (both overloads)
  EXPECT_EQ(plant_->HasJointActuatorNamed("iiwa_joint_3"),
            !param_.remove_joint_actuators);
  EXPECT_EQ(plant_->HasJointActuatorNamed("iiwa_joint_3", arm_model_),
            !param_.remove_joint_actuators);
  // - has_joint_actuator
  const JointActuatorIndex iiwa_joint_3_actuator_index{2};
  EXPECT_EQ(plant_->has_joint_actuator(iiwa_joint_3_actuator_index),
            !param_.remove_joint_actuators);
  // - GetJointActuatorByName (both overloads)
  if (!param_.remove_joint_actuators) {
    EXPECT_EQ(plant_->GetJointActuatorByName("iiwa_joint_3").index(),
              iiwa_joint_3_actuator_index);
    EXPECT_EQ(
        plant_->GetJointActuatorByName("iiwa_joint_3", arm_model_).index(),
        iiwa_joint_3_actuator_index);
  }
  // - GetJointActuatorIndices
  auto get_joint_actuator_indices_names = [&](ModelInstanceIndex model) {
    std::vector<std::string> result;
    for (const auto& i : plant_->GetJointActuatorIndices(model)) {
      result.push_back(plant_->get_joint_actuator(i).name());
    }
    return result;
  };
  if (!param_.remove_joint_actuators) {
    EXPECT_THAT(
        get_joint_actuator_indices_names(arm_model_),
        testing::ElementsAre("iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3",
                             "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6",
                             "iiwa_joint_7"));
  } else {
    EXPECT_THAT(
        get_joint_actuator_indices_names(arm_model_),
        testing::ElementsAre("iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_4",
                             "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"));
  }

  // Check whether ElbowJoint was removed from the acrobot model:
  // - HasJointActuatorNamed (both overloads)
  EXPECT_EQ(plant_->HasJointActuatorNamed("ElbowJoint"),
            !param_.remove_joint_actuators);
  EXPECT_EQ(plant_->HasJointActuatorNamed("ElbowJoint", acrobot_model_),
            !param_.remove_joint_actuators);
  // - has_joint_actuator
  const JointActuatorIndex elbow_joint_actuator_index{7};
  EXPECT_EQ(plant_->has_joint_actuator(elbow_joint_actuator_index),
            !param_.remove_joint_actuators);
  // - GetJointActuatorByName (both overloads)
  if (!param_.remove_joint_actuators) {
    EXPECT_EQ(plant_->GetJointActuatorByName("ElbowJoint").index(),
              elbow_joint_actuator_index);
    EXPECT_EQ(
        plant_->GetJointActuatorByName("ElbowJoint", acrobot_model_).index(),
        elbow_joint_actuator_index);
  }
  if (!param_.remove_joint_actuators) {
    EXPECT_THAT(get_joint_actuator_indices_names(acrobot_model_),
                testing::ElementsAre("ElbowJoint", "ShoulderJoint"));
  } else {
    EXPECT_THAT(get_joint_actuator_indices_names(acrobot_model_),
                testing::ElementsAre("ShoulderJoint"));
  }
}

TEST_P(ActuatedModelsTest, ZeroActuationPriorToStepping) {
  // Load a discrete model, with output sampling enabled.
  constexpr char arm_url[] =
      "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf";
  auto plant = std::make_unique<MultibodyPlant<double>>(0.01);
  Parser parser(plant.get());
  parser.AddModelsFromUrl(arm_url);
  plant->Finalize();

  // Provide actuation input.
  auto model_instance = plant->GetModelInstanceByName("iiwa7");
  auto context = plant->CreateDefaultContext();
  plant->get_actuation_input_port(model_instance)
      .FixValue(context.get(), VectorXd::Constant(7, 1.0));

  // Evaluate all of the actuation output ports to confirm they are zero.
  EXPECT_TRUE(plant->get_net_actuation_output_port().Eval(*context).isZero());
  for (ModelInstanceIndex i{0}; i < plant->num_model_instances(); ++i) {
    const auto& ith_port = plant->get_net_actuation_output_port(i);
    EXPECT_TRUE(ith_port.Eval(*context).isZero());
  }

  // Take a step and now they are non-zero.
  plant->ExecuteForcedEvents(context.get());
  EXPECT_FALSE(plant->get_net_actuation_output_port().Eval(*context).isZero());
  const auto& model_port = plant->get_net_actuation_output_port(model_instance);
  EXPECT_FALSE(model_port.Eval(*context).isZero());
}

std::vector<TestParam> MakeAllParams() {
  std::vector<TestParam> result;
  for (const bool remove_joint_actuators : {false, true}) {
    result.push_back(TestParam{
        .remove_joint_actuators = remove_joint_actuators,
    });
  }
  return result;
}

INSTANTIATE_TEST_SUITE_P(All, ActuatedModelsTest,
                         testing::ValuesIn(MakeAllParams()));

}  // namespace
}  // namespace multibody
}  // namespace drake
