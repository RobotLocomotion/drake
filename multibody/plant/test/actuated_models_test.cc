#include <memory>
#include <ostream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/desired_state_input.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/context.h"

using drake::multibody::internal::DesiredStateInput;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {

// MultibodyPlant friend class used to provide access to private methods for
// testing purposes.
class MultibodyPlantTester {
 public:
  static const VectorXd& EvalActuationInput(const MultibodyPlant<double>& plant,
                                            const Context<double>& context,
                                            bool apply_effort_limit) {
    return plant.EvalActuationInput(context, apply_effort_limit);
  }

  static const DesiredStateInput<double>& EvalDesiredStateInput(
      const MultibodyPlant<double>& plant, const Context<double>& context) {
    return plant.EvalDesiredStateInput(context);
  }
};

namespace {

// The tests in this file are all parameterized by these options, which we sweep
// over all useful permutations. (See MakeAllParams(), below.)
struct TestParam {
  template <typename Archive>
  void Serialize(Archive* a) {
    // We intentionally omit `param_index` here, since it's already printed.
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(discrete_contact_approximation));
    a->Visit(DRAKE_NVP(remove_joint_actuators));
    a->Visit(DRAKE_NVP(use_pd_control));
    a->Visit(DRAKE_NVP(full_pd_control));
    a->Visit(DRAKE_NVP(use_joint_locking));
  }

  // Zero-based index into the list of params to sweep over. This suffix is
  // appended to the test case name (e.g., ActuatedModelsTest.GetSetPdGains/0).
  int param_index{};

  double time_step{};
  std::string discrete_contact_approximation;
  bool remove_joint_actuators{};
  bool use_pd_control{};   // Whether any PD control is used at all.
  bool full_pd_control{};  // Whether *all* actuators have PD control.
  bool use_joint_locking{};
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
//
// If `full_pd_control` is set (in which case `use_pd_control` must also be
// set), then all actuators have a PD controller defined.  Otherwise, if only
// `use_pd_control` is set, then half of the arm's actuators have a controller
// as well as all of the gripper's actuators.
//
// If `use_joint_locking` is set, then one joint on the arm will be locked.
class ActuatedModelsTest : public ::testing::TestWithParam<TestParam> {
 public:
  ActuatedModelsTest() : param_(GetParam()) {}

  // By default, finalizes the plant (but callers can opt-out).
  // By default, uses unsampled mode (we're not stepping time).
  // Uses the `gains_` member field to configure gains
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
    plant_->WeldFrames(plant_->world_frame(), iiwa_link_0.body_frame());
    const auto& iiwa_link_7 = plant_->GetBodyByName("iiwa_link_7", arm_model_);
    const auto& gripper_body = plant_->GetBodyByName("body", gripper_model_);
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

    // Finalize, set a default state, and optionally configure joint locking.
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

      if (param_.use_joint_locking) {
        plant_->GetJointByName("iiwa_joint_2").Lock(context_.get());
      }
    }

    // Configure PD gains. (See class overview for details.)
    if (param_.use_pd_control) {
      std::vector<ModelInstanceIndex> pd_models{arm_model_, gripper_model_};
      if (param_.full_pd_control) {
        pd_models.insert(++pd_models.begin(), acrobot_model_);
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
      bool iiwa_within_limits) {
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
      const VectorXd gripper_u = VectorXd::LinSpaced(2, 1.0, 2.0);
      // ElbowJoint is removed from the model.
      const Vector1d acrobot_u(3.0);
      return std::make_tuple(arm_u, acrobot_u, gripper_u);
    } else {
      const VectorXd arm_u =
          iiwa_within_limits
              ? (VectorXd(7) << 50, 40, 55, -35, -40, -35, -40).finished()
              : (VectorXd(7) << 200, 200, 55, -160, -120, -45, -45).finished();
      const VectorXd gripper_u = VectorXd::LinSpaced(2, 1.0, 2.0);
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

  // Helper that calculates either:
  // - in discrete time: the x_next state if we were to take a step,
  // - in continuous time: the x' derivatives.
  // In both cases, the return value is affected by input port forces.
  VectorXd CalcStep() const {
    if (plant_->is_discrete()) {
      auto updates = plant_->AllocateDiscreteVariables();
      plant_->CalcForcedDiscreteVariableUpdate(*context_, updates.get());
      return updates->get_vector().CopyToVector();
    } else {
      return plant_->EvalTimeDerivatives(*context_).CopyToVector();
    }
  }

 protected:
  static constexpr double kProportionalGain{10000.0};
  static constexpr double kDerivativeGain{100.0};

  const TestParam param_;
  PdControllerGains gains_{
      // These are the gains used by SetUpModel(). They are not used afterward.
      .p = kProportionalGain,
      .d = kDerivativeGain,
  };

  std::unique_ptr<MultibodyPlant<double>> plant_;
  ModelInstanceIndex arm_model_;
  ModelInstanceIndex acrobot_model_;
  ModelInstanceIndex gripper_model_;
  ModelInstanceIndex box_model_;
  std::unique_ptr<Context<double>> context_;
};

// Check input ports and their sizes. All model instances have an actuation
// input port and desired state input port, even for unactuated models.
TEST_P(ActuatedModelsTest, PortAndDofSizes) {
  // Joint locking is not relevant; nothing here is context-dependent.
  // We'll skip it to save test time.
  if (param_.use_joint_locking) {
    return;
  }
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
  // Joint locking is not relevant; nothing here is context-dependent.
  // We'll skip it to save test time.
  if (param_.use_joint_locking) {
    return;
  }

  // For this test, we need to start from a clean slate (no gains).
  if (param_.use_pd_control) {
    return;
  }
  SetUpModel();

  // Set and clear gains, one model instance at a time.
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

  for (const bool apply_effort_limit : {false, true}) {
    // Arbitrary expected actuation.
    const VectorXd arm_u = VectorXd::Zero(expected_num_actuators(arm_model_));
    const VectorXd acrobot_u =
        VectorXd::LinSpaced(expected_num_actuators(acrobot_model_), 1.0, 2.0);
    const VectorXd gripper_u =
        VectorXd::LinSpaced(expected_num_actuators(gripper_model_), 90.0, 91.0);

    // We leave the arm's port disconnected to verify its value defaults to
    // zero.
    plant_->get_actuation_input_port(acrobot_model_)
        .FixValue(context_.get(), acrobot_u);
    plant_->get_actuation_input_port(gripper_model_)
        .FixValue(context_.get(), gripper_u);

    // Check the input port assembly.
    const VectorXd expected_gripper_u =
        apply_effort_limit
            ? VectorXd::Constant(expected_num_actuators(gripper_model_), 80.0)
            : gripper_u;
    const VectorXd full_u = MultibodyPlantTester::EvalActuationInput(
        *plant_, *context_, apply_effort_limit);
    const VectorXd expected_u =
        AssembleFullModelActuation(arm_u, acrobot_u, expected_gripper_u);
    EXPECT_EQ(full_u, expected_u);
  }
}

// Verify the assembly of desired state input ports. In particular, we check for
// precise rejection of NaN values.
TEST_P(ActuatedModelsTest, EvalDesiredStateInput) {
  SetUpModel();
  std::unordered_set<ModelInstanceIndex> pd_models;
  if (param_.use_pd_control) {
    pd_models.insert(arm_model_);
    if (param_.full_pd_control) {
      pd_models.insert(acrobot_model_);
    }
    pd_models.insert(gripper_model_);
  }

  int expected_num_desired_states = 0;
  for (const auto& model : {arm_model_, acrobot_model_, gripper_model_}) {
    const auto& port = plant_->get_desired_state_input_port(model);
    const int nu = expected_num_actuators(model);

    // Set the model's desired state input to all NaN. If any actuator in this
    // model was PD-controlled, we should see an exception.
    port.FixValue(context_.get(), VectorXd::Constant(2 * nu, NAN));
    if (pd_models.count(model) > 0) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_),
          "Desired state input port .*NaN.");
    } else {
      EXPECT_NO_THROW(
          MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));
    }

    // Set the model's desired state input to arbitrary valid values and expect
    // no exceptions. We feed a NaN into a non-PD-controlled actuator (if there
    // is one) to confirm that it's ignored, and likewise a locked joint.
    VectorXd xd = VectorXd::LinSpaced(2 * nu, 1.0, 2.0);
    if (param_.use_pd_control && !param_.full_pd_control) {
      if (model == arm_model_ || model == acrobot_model_) {
        xd[0] = NAN;
      }
    }
    if (param_.use_joint_locking) {
      if (model == arm_model_) {
        xd[1] = NAN;
      }
    }
    port.FixValue(context_.get(), xd);
    EXPECT_NO_THROW(
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_));

    // Amend our tally of expected items.
    if (pd_models.contains(model)) {
      if (model == arm_model_) {
        int expected_num_arm_desired_states = 7;
        if (param_.remove_joint_actuators) {
          // The iiwa_joint_3 is unactuated.
          --expected_num_arm_desired_states;
        }
        if (!param_.full_pd_control) {
          // Only every other actuator is controlled.
          expected_num_arm_desired_states /= 2;
        }
        if (param_.use_joint_locking) {
          // The iiwa_joint_2 is locked.
          --expected_num_arm_desired_states;
        }
        expected_num_desired_states += expected_num_arm_desired_states;
      } else if (model == acrobot_model_) {
        const int expected_num_acrobot_desired_states =
            param_.remove_joint_actuators ? 1 : 2;
        expected_num_desired_states += expected_num_acrobot_desired_states;
      } else {
        DRAKE_DEMAND(model == gripper_model_);
        const int expected_num_gripper_desired_states = 2;
        expected_num_desired_states += expected_num_gripper_desired_states;
      }
    }

    // Check the number of values in the cache entry.
    const DesiredStateInput<double> input =
        MultibodyPlantTester::EvalDesiredStateInput(*plant_, *context_);
    EXPECT_EQ(ssize(input.items), expected_num_desired_states);
  }
}

// This unit test verifies that forces applied through the actuation input port
// have the same effect as applying the same forces using the generalized forces
// input port. We also verify the reported net_actuation output.
TEST_P(ActuatedModelsTest, ActuationMatchesAppliedGeneralizedForces) {
  // We allow PD controllers but set their gains to near-zero (and desired state
  // to current state) since for this test we are only interested on verifying
  // that the effect of input actuation in the dynamics is handled properly.
  gains_.p = 1.0e-10;
  gains_.d = 0.0;
  SetUpModel();

  // Set the desired state input to the current state.
  for (const auto& model : {arm_model_, acrobot_model_, gripper_model_}) {
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
    const VectorXd expected_u =
        AssembleFullModelActuation(arm_u_clamped, acrobot_u, gripper_u);

    // Map actuation to generalized forces.
    const MatrixXd B = plant_->MakeActuationMatrix();
    VectorXd tau = B * expected_u;

    // Input through generalized forces only (actuation input is zero).
    plant_->get_applied_generalized_force_input_port().FixValue(context_.get(),
                                                                tau);
    plant_->get_actuation_input_port(arm_model_)
        .FixValue(context_.get(), VectorXd::Zero(arm_u.size()));
    plant_->get_actuation_input_port(acrobot_model_)
        .FixValue(context_.get(), VectorXd::Zero(acrobot_u.size()));
    plant_->get_actuation_input_port(gripper_model_)
        .FixValue(context_.get(), VectorXd::Zero(gripper_u.size()));
    const VectorXd x_tau = CalcStep();

    // Input through actuation only (generalized force input is zero).
    plant_->get_applied_generalized_force_input_port().FixValue(
        context_.get(), VectorXd::Zero(tau.size()));
    plant_->get_actuation_input_port(arm_model_)
        .FixValue(context_.get(), arm_u);
    plant_->get_actuation_input_port(acrobot_model_)
        .FixValue(context_.get(), acrobot_u);
    plant_->get_actuation_input_port(gripper_model_)
        .FixValue(context_.get(), gripper_u);
    const VectorXd x_actuation = CalcStep();

    // N.B. Generalized forces inputs and actuation inputs feed into the result
    // in very different ways. Actuation input goes through the SAP solver and
    // therefore the accuracy of the solution is affected by solver tolerances.
    const double kTolerance = 1.0e-12;
    EXPECT_TRUE(CompareMatrices(x_actuation, x_tau, kTolerance,
                                MatrixCompareType::relative));

    // Verify the actuation values reported by the plant.
    VerifyNetActuationOutputPorts(arm_u_clamped, acrobot_u, gripper_u,
                                  expected_u, kTolerance);
  }
}

// This unit test verifies that forces induced through the desired state input
// port (i.e., an PD controller) have the same effect as applying the same
// forces using the actuation input port. We also verify the reported
// net_actuation output.
TEST_P(ActuatedModelsTest, PdControlMatchesActuation) {
  // TAMSI doesn't support PD control
  if (param_.discrete_contact_approximation == "tamsi") {
    return;
  }
  // TODO(jwnimmer-tri) Ideally we would check SAP's PD dynamics here as well,
  // but computing the expected PD actuation for a complicated plant like ours
  // is too difficult, because SAP applies the PD constraint to the solved-for
  // state, not the current state. We might need to reformulate how this test
  // works before it can be applied to SAP?
  if (param_.discrete_contact_approximation == "sap") {
    return;
  }
  SetUpModel();

  // Choose reasonable actuation forces. Since the purpose of this test is only
  // to confirm that PD control is effective, we'll zero out any DOFs where PD
  // control is not enabled.
  auto [arm_u, acrobot_u, gripper_u] =
      MakeActuationForEachModel(/* iiwa_within_limits = */ true);
  for (const auto& model : {arm_model_, acrobot_model_, gripper_model_}) {
    VectorXd& model_u = (model == arm_model_)       ? arm_u
                        : (model == acrobot_model_) ? acrobot_u
                                                    : gripper_u;
    int i = 0;
    for (const JointActuatorIndex& actuator_index :
         plant_->GetJointActuatorIndices(model)) {
      const auto& actuator = plant_->get_joint_actuator(actuator_index);
      if (!actuator.has_controller() || actuator.joint().is_locked(*context_)) {
        model_u[i] = 0.0;
      }
      ++i;
    }
  }
  const VectorXd expected_u =
      AssembleFullModelActuation(arm_u, acrobot_u, gripper_u);

  // Input through actuation only (desired state input is not connected).
  plant_->get_actuation_input_port(arm_model_).FixValue(context_.get(), arm_u);
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), acrobot_u);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_u);
  const VectorXd x_actuation = CalcStep();

  // Zero out the actuation input.
  plant_->get_actuation_input_port(arm_model_)
      .FixValue(context_.get(), VectorXd::Zero(arm_u.size()));
  plant_->get_actuation_input_port(acrobot_model_)
      .FixValue(context_.get(), VectorXd::Zero(acrobot_u.size()));
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), VectorXd::Zero(gripper_u.size()));

  // Input through desired state only (actuation input is zero). We want a
  // desired state that causes the same actuation as above; we'll do that by
  // getting 25% from the P term and 75% from the D term:
  //  0.25 u = -Kp * (q0 - qd) ; solve for qd = q0 + (0.25 * u / Kp)
  //  0.75 u = -Kd * (v0 - vd) ; solve for vd = v0 + (0.75 * u / Kd)
  for (const auto& model : {arm_model_, acrobot_model_, gripper_model_}) {
    // The given model's expected net actuation.
    const VectorXd& model_u = (model == arm_model_)       ? arm_u
                              : (model == acrobot_model_) ? acrobot_u
                                                          : gripper_u;
    // The same but with zeros converted to NaNs. Entries will be zero only for
    // actuators without a controller (or where the controller was disabled via
    // joint locking). The NaNs should not affect dynamics.
    const VectorXd tweaked_model_u = model_u.unaryExpr([](double u) {
      return u == 0.0 ? NAN : u;
    });
    // Converts a position or velocity vector ordered by velocity index into a
    // vector ordered by actuation index.
    auto convert_to_actuation_indexing =
        [this, &model](const Eigen::VectorXd& q_or_v) -> VectorXd {
      if (model == acrobot_model_) {
        if (param_.remove_joint_actuators) {
          // ElbowJoint is removed from the model.
          return q_or_v.head(1);
        } else {
          // Actuation order is the opposite of velocity order.
          return q_or_v.reverse();
        }
      }
      if (model == arm_model_ && param_.remove_joint_actuators) {
        // iiwa_joint_3 is removed from the model.
        return (VectorXd(6) << q_or_v.head(2), q_or_v.tail(4)).finished();
      }
      return q_or_v;
    };
    const VectorXd q0_act =
        convert_to_actuation_indexing(plant_->GetPositions(*context_, model));
    const VectorXd v0_act =
        convert_to_actuation_indexing(plant_->GetVelocities(*context_, model));
    const VectorXd qd_act =
        q0_act + (0.25 / kProportionalGain) * tweaked_model_u;
    const VectorXd vd_act =  //
        v0_act + (0.75 / kDerivativeGain) * tweaked_model_u;
    const VectorXd xd_act =
        (VectorXd(2 * qd_act.size()) << qd_act, vd_act).finished();
    plant_->get_desired_state_input_port(model).FixValue(context_.get(),
                                                         xd_act);
  }
  const VectorXd x_desired_state = CalcStep();

  // Verify that both steps were equivalent.
  const double kTolerance = 1e-12;
  EXPECT_TRUE(CompareMatrices(x_desired_state, x_actuation, kTolerance,
                              MatrixCompareType::relative));

  // Verify the actuation values reported by the plant.
  VerifyNetActuationOutputPorts(arm_u, acrobot_u, gripper_u, expected_u,
                                kTolerance);
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

// Sweep through all sensible permutations of options.
std::vector<TestParam> MakeAllParams() {
  std::vector<TestParam> result;
  int param_index = 0;
  for (const double time_step : {0.01, 0.0}) {
    for (const auto& discrete_contact_approximation :
         (time_step > 0.0) ? std::vector<std::string>{"sap", "tamsi"}
                           : std::vector<std::string>{""}) {
      for (const bool remove_joint_actuators : {false, true}) {
        for (int pd_case = 0; pd_case < 3; ++pd_case) {
          const bool use_pd_control = pd_case >= 1;
          const bool full_pd_control = pd_case >= 2;
          for (const bool use_joint_locking : {false, true}) {
            result.push_back(TestParam{
                .param_index = param_index,
                .time_step = time_step,
                .discrete_contact_approximation =
                    discrete_contact_approximation,
                .remove_joint_actuators = remove_joint_actuators,
                .use_pd_control = use_pd_control,
                .full_pd_control = full_pd_control,
                .use_joint_locking = use_joint_locking,
            });
            ++param_index;
          }
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
