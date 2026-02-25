#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
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
  template <typename Archive>
  void Serialize(Archive* a) {
    // We intentionally omit `param_index` here, since it's already printed.
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(discrete_contact_approximation));
    a->Visit(DRAKE_NVP(remove_joint_actuators));
    a->Visit(DRAKE_NVP(use_pd_control));
    a->Visit(DRAKE_NVP(full_pd_control));
  }

  // Zero-based index into the list of params to sweep over. This suffix is
  // appended to the test case name (e.g., ActuatedModelsTest.GetSetPdGains/0).
  int param_index{};

  double time_step{};
  std::string discrete_contact_approximation;
  bool remove_joint_actuators{};
  bool use_pd_control{};   // Whether any PD control is used at all.
  bool full_pd_control{};  // Whether *all* actuators have PD control.

  // - discrete time unsampled vs sampled
  // - actuation input over limit yes/no
  // - desired state is connected yes/no
  // - joint is locked
};

// Allow googletest to print the TestParam.
std::ostream& operator<<(std::ostream& os, const TestParam& param) {
  return os << yaml::SaveJsonString(param);
}

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

  // By default, finalizes the plant (but callers can opt-out).
  // By default, uses unsampled mode We're not stepping time.
  void SetUpModel(const bool finalize = true,
                  const bool sampled_output_ports = false) {
    // Construct the plant in the requested mode.
    const MultibodyPlantConfig config{
        .time_step = param_.time_step,
        .discrete_contact_approximation =
            param_.discrete_contact_approximation};
    plant_ = std::make_unique<MultibodyPlant<double>>(config.time_step);
    ApplyMultibodyPlantConfig(config, plant_.get());
    plant_->SetUseSampledOutputPorts(sampled_output_ports);

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

      const VectorXd arm_q0 =
          (VectorXd(7) << 0, 0, 0, -1.7, 0, 1.0, 0).finished();
      const VectorXd arm_v0 = VectorXd::Zero(7);
      const VectorXd arm_x0 = (VectorXd(14) << arm_q0, arm_v0).finished();
      const VectorXd gripper_q0 = VectorXd::Zero(2);
      const VectorXd gripper_v0 = VectorXd::Zero(2);
      const VectorXd gripper_x0 =
          (VectorXd(4) << gripper_q0, gripper_v0).finished();

      plant_->SetPositionsAndVelocities(context_.get(), arm_model_, arm_x0);
      plant_->SetPositionsAndVelocities(context_.get(), gripper_model_,
                                        gripper_x0);
    }

    // Configure PD gains. When full_pd_control is true, all actuators have a PD
    // controller. When false, only the arm and gripper are controlled, and
    // within the arm only odd-count actuators are controlled.
    if (param_.use_pd_control) {
      std::vector<ModelInstanceIndex> pd_models{arm_model_, gripper_model_};
      if (param_.full_pd_control) {
        pd_models.insert(++pd_models.begin(), arm_model_);
      }
      for (const auto& model : pd_models) {
        int count = 0;
        for (const auto& i : plant_->GetJointActuatorIndices(model)) {
          auto& actuator = plant_->get_mutable_joint_actuator(i);
          const bool odd_only =
              !param_.full_pd_control && (model == arm_model_);
          if (odd_only && (count % 2 == 0)) {
            // We skip the 0th, 2nd, etc. actuators within this model instance.
          } else {
            actuator.set_controller_gains(gains_);
          }
          ++count;
        }
      }
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
  static constexpr double kProportionalGain{10000.0};
  static constexpr double kDerivativeGain{100.0};

  const TestParam param_;
  PdControllerGains gains_{
      .p = kProportionalGain,
      .d = kDerivativeGain,
  };

  std::unique_ptr<MultibodyPlant<double>> plant_;
  ModelInstanceIndex arm_model_;
  ModelInstanceIndex acrobot_model_;
  ModelInstanceIndex gripper_model_;
  ModelInstanceIndex box_model_;
  std::unique_ptr<Context<double>> context_;

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

// Check input ports and their sizes. All model instances have an actuation
// input port and desired state input port, even for unactuated models.
TEST_P(ActuatedModelsTest, PortAndDofSizes) {
  SetUpModel();

  int num_actuators_total = 0;
  for (const auto& model :
       {arm_model_, acrobot_model_, gripper_model_, box_model_}) {
    const int expected = expected_num_actuators(model);
    EXPECT_EQ(plant_->num_actuated_dofs(model), expected);
    EXPECT_EQ(plant_->num_actuators(model), expected);
    EXPECT_EQ(plant_->get_actuation_input_port(model).size(), expected);
    EXPECT_EQ(plant_->get_desired_state_input_port(model).size(), 2 * expected);
    EXPECT_EQ(plant_->get_net_actuation_output_port(model).size(), expected);
    num_actuators_total += expected;
  }
  EXPECT_EQ(plant_->num_actuated_dofs(), num_actuators_total);
  EXPECT_EQ(plant_->num_actuators(), num_actuators_total);
  EXPECT_EQ(plant_->get_actuation_input_port().size(), num_actuators_total);
  EXPECT_EQ(plant_->get_net_actuation_output_port().size(),
            num_actuators_total);
}

// Accessing an invalid model instance's input port will throw.
TEST_P(ActuatedModelsTest, InputPortInvalid) {
  // We don't need to run this test case more than once.
  if (param_.param_index > 0) {
    return;
  }
  SetUpModel();

  const ModelInstanceIndex invalid_index(10);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->get_actuation_input_port(invalid_index),
      ".*get_actuation_input_port.*num_model_instances.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->get_desired_state_input_port(invalid_index),
      ".*get_desired_state_input_port.*num_model_instances.*");
}

// Check setting and clearing PD gains.
TEST_P(ActuatedModelsTest, GetSetPdGains) {
  // PD gains are not supported for a continuous-time plant.
  if (param_.time_step == 0.0) {
    SetUpModel(/* finalize = */ false);
    plant_->get_mutable_joint_actuator(JointActuatorIndex{0})
        .set_controller_gains({.p = 1.0});
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant_->Finalize(),
        "Continuous model with PD controlled joint actuators. This feature is "
        "only supported for discrete models. Refer to MultibodyPlant's "
        "documentation for further details.");
    return;
  }
  DRAKE_DEMAND(param_.time_step > 0.0);

  // For this test, we need to start from a clean slate (no gains).
  if (param_.use_pd_control) {
    return;
  }
  SetUpModel();

  for (const auto& model : {arm_model_, acrobot_model_, gripper_model_}) {
    for (const auto& i : plant_->GetJointActuatorIndices(model)) {
      auto& actuator = plant_->get_mutable_joint_actuator(i);
      // Set gains.
      actuator.set_controller_gains({kProportionalGain, kDerivativeGain});
      ASSERT_TRUE(actuator.has_controller());
      EXPECT_EQ(actuator.get_controller_gains().p, kProportionalGain);
      EXPECT_EQ(actuator.get_controller_gains().d, kDerivativeGain);
      // Clear gains.
      actuator.set_controller_gains({0.0, 0.0});
      EXPECT_FALSE(plant_->get_joint_actuator(i).has_controller());
    }
  }
}

// Verify the assembly of actuation input ports. In particular, we verify this
// assembly is performed in the order of JointActuatorIndex and disconnected
// ports default to zero values.
TEST_P(ActuatedModelsTest, EvalActuationInput) {
  SetUpModel();

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

// Verify the assembly of desired states for a plant with two PD controlled
// model instances, one of which is only partially controlled.
TEST_P(ActuatedModelsTest, EvalDesiredStateInput) {
  SetUpModel();

  // Fixed desired state for the gripper.
  const VectorXd gripper_xd =
      VectorXd::LinSpaced(2 * expected_num_actuators(gripper_model_), 1.0, 2.0);
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);

  // We provided no desired state for the arm, and it is therefore "disarmed".
  // Only the gripper is armed.
  const internal::DesiredStateInput<double> input1 =
      MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);
  EXPECT_EQ(input1.num_model_instances(), plant_->num_model_instances());
  EXPECT_FALSE(input1.is_armed(acrobot_model_));
  EXPECT_FALSE(input1.is_armed(arm_model_));
  EXPECT_TRUE(input1.is_armed(gripper_model_));
  EXPECT_FALSE(input1.is_armed(box_model_));
  EXPECT_EQ(input1.positions(gripper_model_),
            gripper_xd.head(gripper_xd.size() / 2));
  EXPECT_EQ(input1.velocities(gripper_model_),
            gripper_xd.tail(gripper_xd.size() / 2));

  // We now do provide desired state for the arm.
  const VectorXd arm_xd =
      VectorXd::LinSpaced(2 * expected_num_actuators(arm_model_), 3.0, 4.0);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  // Now both the arm and gripper are "armed".
  const internal::DesiredStateInput<double> input2 =
      MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);
  EXPECT_EQ(input2.num_model_instances(), plant_->num_model_instances());
  EXPECT_FALSE(input2.is_armed(acrobot_model_));
  EXPECT_TRUE(input2.is_armed(arm_model_));
  EXPECT_TRUE(input2.is_armed(gripper_model_));
  EXPECT_FALSE(input2.is_armed(box_model_));
  EXPECT_EQ(input2.positions(gripper_model_),
            gripper_xd.head(gripper_xd.size() / 2));
  EXPECT_EQ(input2.velocities(gripper_model_),
            gripper_xd.tail(gripper_xd.size() / 2));
  EXPECT_EQ(input2.positions(arm_model_), arm_xd.head(arm_xd.size() / 2));
  EXPECT_EQ(input2.velocities(arm_model_), arm_xd.tail(arm_xd.size() / 2));
}

// Verify that an exception is thrown if there are NaNs in the desired state
// input port, only for states corresponding to PD-controlled actuators. Desired
// states for non PD-controlled actuators are ignored.
TEST_P(ActuatedModelsTest, EvalDesiredStateInput_RejectNansUnlessIgnored) {
  SetUpModel();

  // With no PD control, NaNs are allows for all desired state input.
  // After we check that, we can bail out early on the remainder of the tests.
  if (!param_.use_pd_control) {
    for (const auto& model : {arm_model_, acrobot_model_, gripper_model_}) {
      const auto& port = plant_->get_desired_state_input_port(model);
      port.FixValue(context_.get(), VectorXd::Constant(port.size(), NAN));
    }
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
    return;
  }

  // Purposely inject NaN values for actuators with no PD. These should not
  // trigger an exception since they are ignored.
  if (!param_.full_pd_control) {
    const int num_u = expected_num_actuators(arm_model_);
    VectorXd arm_xd = VectorXd::LinSpaced(2 * num_u, 1.0, 2.0);
    // First actuator does not have PD-control.
    const int first_actuator_index = 0;
    const JointActuator<double>& actuator = plant_->get_joint_actuator(
        plant_->GetJointActuatorIndices(arm_model_)[first_actuator_index]);
    ASSERT_FALSE(actuator.has_controller());
    arm_xd[first_actuator_index] = NAN;
    arm_xd[num_u + first_actuator_index] = NAN;
    plant_->get_desired_state_input_port(arm_model_)
        .FixValue(context_.get(), arm_xd);
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
  }

  // NaN values for PD-controlled actuators do trigger an exception, unless the
  // actuated joint is locked.
  {
    const int num_u = expected_num_actuators(arm_model_);
    VectorXd arm_xd = VectorXd::LinSpaced(2 * num_u, 1.0, 2.0);
    // Second actuator does have PD-control.
    const JointActuatorIndex pd_actuator(1);
    const auto& actuator = plant_->get_joint_actuator(pd_actuator);
    ASSERT_TRUE(actuator.has_controller());

    // NaN qd, valid vd.
    arm_xd[pd_actuator] = NAN;
    arm_xd[num_u + pd_actuator] = 0.0;
    plant_->get_desired_state_input_port(arm_model_)
        .FixValue(context_.get(), arm_xd);
    DRAKE_EXPECT_THROWS_MESSAGE(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_),
        "Desired state input port for model instance iiwa7 contains NaN.");
    // The NaN should be ignored if the actuated joint is locked.
    actuator.joint().Lock(context_.get());
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
    actuator.joint().Unlock(context_.get());

    // Valid qd, NaN vd.
    arm_xd[pd_actuator] = 0.0;
    arm_xd[num_u + pd_actuator] = NAN;
    plant_->get_desired_state_input_port(arm_model_)
        .FixValue(context_.get(), arm_xd);
    DRAKE_EXPECT_THROWS_MESSAGE(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_),
        "Desired state input port for model instance iiwa7 contains NaN.");
    // The NaN should be ignored if the actuated joint is locked.
    actuator.joint().Lock(context_.get());
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
    actuator.joint().Unlock(context_.get());
  }
}

// This unit test verifies that, when within effort limits, forces applied
// through the generalized forces input port has the same effect as applying the
// same forces using the actuation input port.
TEST_P(ActuatedModelsTest, ActuationMatchesAppliedGeneralizedForces) {
  if (param_.time_step == 0.0) {
    // We use CalcForcedDiscreteVariableUpdate to compare the step forces, but
    // that method is not available for a continuous-time plant.
    return;
  }
  // We add PD controllers but set their gains to near-zero (and desired state
  // to current state) since for this test we are only interested on verifying
  // that the effect of input actuation in the dynamics is handled properly.
  gains_.p = 1.0e-10;
  gains_.d = 0.0;
  SetUpModel();

  const Eigen::SparseMatrix<double> Binv =
      plant_->MakeActuationMatrixPseudoinverse();
  for (const auto& model : {arm_model_, acrobot_model_, gripper_model_}) {
    // Set the desired state input to the current state.
    const auto& full_q = plant_->GetPositions(*context_);
    const auto& full_v = plant_->GetVelocities(*context_);
    const int num_u = expected_num_actuators(model);
    VectorXd x_desired = VectorXd::Zero(2 * num_u);
    int i = 0;
    for (const JointActuatorIndex& actuator_index :
         plant_->GetJointActuatorIndices(model)) {
      const auto& joint = plant_->get_joint_actuator(actuator_index).joint();
      x_desired.head(num_u)[i] = full_q[joint.position_start()];
      x_desired.tail(num_u)[i] = full_v[joint.velocity_start()];
      ++i;
    }
    plant_->get_desired_state_input_port(model).FixValue(context_.get(),
                                                         x_desired);
  }

  for (const bool iiwa_within_limits : {true, false}) {
    SCOPED_TRACE(fmt::format("iiwa_within_limits = {}", iiwa_within_limits));

    auto [arm_u, acrobot_u, gripper_u] =
        MakeActuationForEachModel(iiwa_within_limits);

    // We might need to clamp u to be within the iiwa effort limits so that we
    // can make an equivalent vector or generalized forces below.
    const VectorXd limits =
        param_.remove_joint_actuators
            ? (VectorXd(6) << 176, 176, 110, 110, 40, 40).finished()
            : (VectorXd(7) << 176, 176, 110, 110, 110, 40, 40).finished();
    const VectorXd arm_u_clamped =
        arm_u.array().min(limits.array()).max(-limits.array());
    const VectorXd expected_u = AssembleFullModelActuation(
        iiwa_within_limits ? arm_u : arm_u_clamped, acrobot_u, gripper_u);

    // Map actuation to generalized forces.
    const MatrixXd B = plant_->MakeActuationMatrix();
    VectorXd tau = B * expected_u;

    auto updates = plant_->AllocateDiscreteVariables();

    // Input through generalized forces only (actuation input is zero).
    plant_->get_applied_generalized_force_input_port().FixValue(context_.get(),
                                                                tau);
    plant_->get_actuation_input_port(arm_model_)
        .FixValue(context_.get(), VectorXd::Zero(arm_u.size()));
    plant_->get_actuation_input_port(acrobot_model_)
        .FixValue(context_.get(), VectorXd::Zero(acrobot_u.size()));
    plant_->get_actuation_input_port(gripper_model_)
        .FixValue(context_.get(), VectorXd::Zero(gripper_u.size()));
    plant_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());
    const VectorXd x_tau = updates->get_vector().CopyToVector();

    // Input through actuation only (generalized force input is zero).
    plant_->get_applied_generalized_force_input_port().FixValue(
        context_.get(), VectorXd::Zero(tau.size()));
    plant_->get_actuation_input_port(arm_model_)
        .FixValue(context_.get(), arm_u);
    plant_->get_actuation_input_port(acrobot_model_)
        .FixValue(context_.get(), acrobot_u);
    plant_->get_actuation_input_port(gripper_model_)
        .FixValue(context_.get(), gripper_u);
    plant_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());
    const VectorXd x_actuation = updates->get_vector().CopyToVector();

    // N.B. Generalized forces inputs and actuation inputs feed into the result
    // in very different ways. Actuation input goes through the SAP solver and
    // therefore the accuracy of the solution is affected by solver tolerances.
    const double kTolerance = 1.0e-12;
    EXPECT_TRUE(CompareMatrices(x_actuation, x_tau, kTolerance,
                                MatrixCompareType::relative));

    // Verify the actuation values reported by the plant.
    {
      SCOPED_TRACE(fmt::format("net actuation, nominal"));
      VerifyNetActuationOutputPorts(arm_u_clamped, acrobot_u, gripper_u,
                                    expected_u, kTolerance);
    }

    // Locking a joint disables the (non-existent) PD controller, but otherwise
    // doesn't change anything. In particular, the effort limit is still
    // enforced.
    {
      SCOPED_TRACE(fmt::format("net actuation, locked"));
      plant_->GetJointByName("iiwa_joint_4").Lock(context_.get());
      VerifyNetActuationOutputPorts(arm_u_clamped, acrobot_u, gripper_u,
                                    expected_u, kTolerance);
    }
  }
}

// This test verifies that the actuation output port simply feeds through the
// actuation inputs.
TEST_P(ActuatedModelsTest, ActuationOutputFeedsThroughActuationInput) {
  SetUpModel();

  // Set arbitrary actuation values.
  auto [arm_u, acrobot_u, gripper_u] =
      MakeActuationForEachModel(true /* iiwa inside limits */);
  plant_->get_actuation_input_port(arm_model_).FixValue(context_.get(), arm_u);
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), acrobot_u);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_u);

  // Verify that net actuation output is an exact copy of the inputs.
  const VectorXd full_u =
      AssembleFullModelActuation(arm_u, acrobot_u, gripper_u);
  VerifyNetActuationOutputPorts(arm_u, acrobot_u, gripper_u, full_u);
}

TEST_P(ActuatedModelsTest, ZeroActuationPriorToStepping) {
  // Testing stepping is not relevant on a continuous-time plant.
  if (param_.time_step == 0.0) {
    return;
  }

  // Load a discrete model, with output sampling enabled.
  SetUpModel(/* finalize = */ true,
             /* sampled_output_ports = */ true);

  // Provide actuation input.
  auto context = plant_->CreateDefaultContext();
  plant_->get_actuation_input_port(arm_model_)
      .FixValue(context.get(),
                VectorXd::Constant(expected_num_actuators(arm_model_), 1.0));

  // Evaluate all of the actuation output ports to confirm they are zero.
  EXPECT_TRUE(plant_->get_net_actuation_output_port().Eval(*context).isZero());
  for (ModelInstanceIndex i{0}; i < plant_->num_model_instances(); ++i) {
    const auto& ith_port = plant_->get_net_actuation_output_port(i);
    EXPECT_TRUE(ith_port.Eval(*context).isZero());
  }

  // Take a step and now they are non-zero.
  plant_->ExecuteForcedEvents(context.get());
  EXPECT_FALSE(plant_->get_net_actuation_output_port().Eval(*context).isZero());
  const auto& model_port = plant_->get_net_actuation_output_port(arm_model_);
  EXPECT_FALSE(model_port.Eval(*context).isZero());
}

std::vector<TestParam> MakeAllParams() {
  std::vector<TestParam> result;
  int param_index = 0;
  for (const double time_step : {0.01, 0.0}) {
    for (const auto& discrete_contact_approximation :
         (time_step > 0.0) ? std::vector<std::string>{"sap", "tamsi"}
                           : std::vector<std::string>{""}) {
      for (const bool remove_joint_actuators : {false, true}) {
        for (int pd_case = 0; pd_case < 3; ++pd_case) {
          if (time_step == 0.0 && pd_case > 0) {
            // PD gains are not supported for a continuous-time plant.
            continue;
          }
          const bool use_pd_control = pd_case >= 1;
          const bool full_pd_control = pd_case >= 2;
          result.push_back(TestParam{
              .param_index = param_index,
              .time_step = time_step,
              .discrete_contact_approximation = discrete_contact_approximation,
              .remove_joint_actuators = remove_joint_actuators,
              .use_pd_control = use_pd_control,
              .full_pd_control = full_pd_control});
          ++param_index;
        }
      }
    }
  }
  return result;
}

INSTANTIATE_TEST_SUITE_P(All, ActuatedModelsTest,
                         testing::ValuesIn(MakeAllParams()));

}  // namespace
}  // namespace multibody
}  // namespace drake
