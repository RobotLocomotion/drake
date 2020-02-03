#include "drake/manipulation/util/robot_state_msg_translator.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

namespace drake {

using multibody::joints::FloatingBaseType;

namespace manipulation {

namespace {
constexpr int kNumTests = 100;
}  // namespace

class RobotStateLcmMessageTranslatorTest : public ::testing::Test {
 protected:
  void SetUp() override { rand_.seed(1234); }

  void Initialize(const std::string& model, bool is_urdf,
                  std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
                  FloatingBaseType floating_base_type) {
    base_type_ = floating_base_type;
    robot_ = std::make_unique<RigidBodyTree<double>>();
    if (is_urdf) {
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          model, floating_base_type, weld_to_frame, robot_.get());
    } else {
      parsers::sdf::AddModelInstancesFromSdfFile(model, floating_base_type,
                                                 weld_to_frame, robot_.get());
    }

    dut_ = std::make_unique<RobotStateLcmMessageTranslator>(*robot_);
    dut_->InitializeMessage(&message_);
  }

  // Tests encode and decode for various floating base types.
  void RunEncodeDecode() {
    VectorX<double> q, v, torque, q_expected, v_expected, torque_expected;
    q_expected.resize(robot_->get_num_positions());
    v_expected.resize(robot_->get_num_velocities());
    torque_expected.resize(robot_->get_num_actuators());

    // Finds the first index into q v that is after the floating base.
    int q_non_floating_joint_start_index = 0;
    int v_non_floating_joint_start_index = 0;
    switch (base_type_) {
      case FloatingBaseType::kRollPitchYaw:
        q_non_floating_joint_start_index = 6;
        v_non_floating_joint_start_index = 6;
        break;
      case FloatingBaseType::kQuaternion:
      case FloatingBaseType::kExperimentalMultibodyPlantStyle:
        q_non_floating_joint_start_index = 7;
        v_non_floating_joint_start_index = 6;
        break;
      case FloatingBaseType::kFixed:
        q_non_floating_joint_start_index = 0;
        v_non_floating_joint_start_index = 0;
        break;
    }

    for (int n = 0; n < kNumTests; ++n) {
      SetRandomQVTorque(&q, &v, &torque);

      dut_->EncodeMessageKinematics(q, v, &message_);
      dut_->EncodeMessageTorque(torque, &message_);

      // Tests encode.
      // Checks that encoded joint position and velocity are in q order (after
      // the floating joints).
      for (int i = 0; i < message_.num_joints; ++i) {
        const int q_idx = i + q_non_floating_joint_start_index;
        const int v_idx = i + v_non_floating_joint_start_index;
        EXPECT_EQ(robot_->get_position_name(q_idx), message_.joint_name[i]);
        EXPECT_NEAR(q[q_idx], message_.joint_position[i], tolerance_);
        EXPECT_NEAR(v[v_idx], message_.joint_velocity[i], tolerance_);
      }

      // Checks that encoded joint torques are in the same order.
      for (size_t i = 0; i < robot_->actuators.size(); i++) {
        const auto& actuator = robot_->actuators[i];
        const auto& body = *actuator.body_;
        const int q_idx = body.get_position_start_index();
        const int actuator_idx = q_idx - q_non_floating_joint_start_index;
        EXPECT_NEAR(torque[i], message_.joint_effort[actuator_idx], tolerance_);
      }

      // Checks that the pose and velocity matches X_WB and V_WB.
      const RigidBodyTree<double>& robot = dut_->get_robot();
      const RigidBody<double>& base = robot.get_body(1);
      KinematicsCache<double> cache = robot.doKinematics(q, v);

      Isometry3<double> X_WB = DecodePose(message_.pose);
      Vector6<double> V_WB = DecodeTwist(message_.twist);

      Isometry3<double> X_WB_expected =
          robot.CalcBodyPoseInWorldFrame(cache, base);
      Vector6<double> V_WB_expected =
          robot.CalcBodySpatialVelocityInWorldFrame(cache, base);

      EXPECT_TRUE(CompareMatrices(X_WB_expected.matrix(), X_WB.matrix(),
                                  tolerance_, MatrixCompareType::absolute));
      EXPECT_TRUE(CompareMatrices(V_WB_expected, V_WB, tolerance_,
                                  MatrixCompareType::absolute));

      // Tests decode.
      dut_->DecodeMessageKinematics(message_, q_expected, v_expected);
      dut_->DecodeMessageTorque(message_, torque_expected);

      // Checks q.
      if (base_type_ == FloatingBaseType::kFixed ||
          base_type_ == FloatingBaseType::kRollPitchYaw) {
        EXPECT_TRUE(CompareMatrices(q_expected, q, tolerance_,
                                    MatrixCompareType::absolute));
      } else if (base_type_ ==
                 FloatingBaseType::kQuaternion) {
        // Checks for -quat = quat.
        VectorX<double> q_expected_w_neg_quat = q_expected;
        q_expected_w_neg_quat.segment<4>(3) *= -1;
        bool equal = CompareMatrices(q_expected, q, tolerance_,
                                     MatrixCompareType::absolute) ||
                     CompareMatrices(q_expected_w_neg_quat, q, tolerance_,
                                     MatrixCompareType::absolute);
        EXPECT_TRUE(equal);
      }

      // Checks v.
      EXPECT_TRUE(CompareMatrices(v_expected, v, tolerance_,
                                  MatrixCompareType::absolute));

      // Checks torque.
      EXPECT_TRUE(CompareMatrices(torque_expected, torque, tolerance_,
                                  MatrixCompareType::absolute));
    }
  }

  // Sets q, v, and torque to some random value.
  void SetRandomQVTorque(VectorX<double>* q, VectorX<double>* v,
                         VectorX<double>* torque) {
    std::normal_distribution<double> distribution;
    v->resize(robot_->get_num_velocities());
    torque->resize(robot_->get_num_actuators());
    *q = robot_->getRandomConfiguration(rand_);
    for (int i = 0; i < robot_->get_num_velocities(); i++) {
      (*v)[i] = distribution(rand_);
    }
    for (int i = 0; i < robot_->get_num_actuators(); i++) {
      (*torque)[i] = distribution(rand_);
    }
  }

  FloatingBaseType base_type_;
  bot_core::robot_state_t message_{};
  // Same seed every time, but that's OK.
  std::default_random_engine rand_;
  std::unique_ptr<RobotStateLcmMessageTranslator> dut_;
  std::unique_ptr<RigidBodyTree<double>> robot_;
  // Low tolerance because bot_core::robot_state_t uses float for storage.
  const double tolerance_{10. * std::numeric_limits<float>::epsilon()};
};

// Tests encoding and decoding message for
// {iiwa} x {RPY, quaterion, fixed based}
// x {zero base offset, non zero base offset}.
TEST_F(RobotStateLcmMessageTranslatorTest, TestEncodeDecode) {
  const std::string model =
      FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/urdf/"
          "iiwa14_polytope_collision.urdf");

  std::vector<FloatingBaseType> floating_types = {
    FloatingBaseType::kFixed,
    FloatingBaseType::kRollPitchYaw,
    FloatingBaseType::kQuaternion};

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "world", nullptr,
      Vector3<double>(1, 2, 3), Vector3<double>(0.1, -0.3, 0.6));

  for (const auto& type : floating_types) {
    Initialize(model, true, nullptr, type);
    RunEncodeDecode();

    Initialize(model, true, weld_to_frame, type);
    RunEncodeDecode();
  }
}

// Tests an underactuated robot.
TEST_F(RobotStateLcmMessageTranslatorTest, TestUnderActuated) {
  std::string model = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
  std::vector<FloatingBaseType> types = {
      FloatingBaseType::kFixed,
      FloatingBaseType::kQuaternion,
      FloatingBaseType::kFixed};

  for (const auto& type : types) {
    Initialize(model, false, nullptr, type);
    RunEncodeDecode();
  }
}

// Tests RobotStateLcmMessageTranslator::CheckMessageVectorSize()
TEST_F(RobotStateLcmMessageTranslatorTest, TestSizeCheck) {
  std::string model = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  Initialize(model, true, nullptr,
      FloatingBaseType::kFixed);

  message_.num_joints++;
  EXPECT_FALSE(
      RobotStateLcmMessageTranslator::CheckMessageVectorSize(message_));
  message_.num_joints--;

  message_.joint_name.push_back("aa");
  EXPECT_FALSE(
      RobotStateLcmMessageTranslator::CheckMessageVectorSize(message_));
  message_.joint_name.pop_back();

  message_.joint_position.push_back(0);
  EXPECT_FALSE(
      RobotStateLcmMessageTranslator::CheckMessageVectorSize(message_));
  message_.joint_position.pop_back();

  message_.joint_velocity.push_back(0);
  EXPECT_FALSE(
      RobotStateLcmMessageTranslator::CheckMessageVectorSize(message_));
  message_.joint_velocity.pop_back();

  message_.joint_effort.push_back(0);
  EXPECT_FALSE(
      RobotStateLcmMessageTranslator::CheckMessageVectorSize(message_));
}

TEST_F(RobotStateLcmMessageTranslatorTest, TestEncodeDecodeExtraJointNames) {
  std::string model = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  // Since this test only affects the non floating base joints, so fixed base
  // is sufficient.
  Initialize(model, true, nullptr,
      FloatingBaseType::kFixed);

  VectorX<double> q, v, torque;
  SetRandomQVTorque(&q, &v, &torque);

  // Add some random joint to the message.
  message_.num_joints++;
  const int kExtraIndex = 2;
  message_.joint_name.insert(message_.joint_name.begin() + kExtraIndex,
                             "bilibili");
  message_.joint_position.insert(message_.joint_position.begin() + kExtraIndex,
                                 233);
  message_.joint_velocity.insert(message_.joint_velocity.begin() + kExtraIndex,
                                 233);
  message_.joint_effort.insert(message_.joint_effort.begin() + kExtraIndex,
                               233);

  SetRandomQVTorque(&q, &v, &torque);
  dut_->EncodeMessageKinematics(q, v, &message_);
  dut_->EncodeMessageTorque(torque, &message_);

  EXPECT_EQ(robot_->get_num_positions(), message_.num_joints - 1);
  for (int i = 0; i < robot_->get_num_positions(); ++i) {
    int idx = i;
    if (i >= kExtraIndex) idx++;

    EXPECT_EQ(robot_->get_position_name(i), message_.joint_name[idx]);
    EXPECT_NEAR(q[i], message_.joint_position[idx], tolerance_);
    EXPECT_NEAR(v[i], message_.joint_velocity[idx], tolerance_);
    EXPECT_NEAR(torque[i], message_.joint_effort[idx], tolerance_);
  }

  // Should leave the unknown joint as is.
  EXPECT_EQ(message_.joint_name[kExtraIndex], "bilibili");
  EXPECT_EQ(message_.joint_position[kExtraIndex], 233);
  EXPECT_EQ(message_.joint_velocity[kExtraIndex], 233);
  EXPECT_EQ(message_.joint_effort[kExtraIndex], 233);

  // Similar test for decode, this should have no effect on decode, since the
  // additional information is discarded.
  VectorX<double> q_expected =
      VectorX<double>::Ones(robot_->get_num_positions());
  VectorX<double> v_expected =
      VectorX<double>::Ones(robot_->get_num_velocities());
  VectorX<double> torque_expected =
      VectorX<double>::Ones(robot_->get_num_actuators());
  dut_->DecodeMessageKinematics(message_, q_expected, v_expected);
  dut_->DecodeMessageTorque(message_, torque_expected);

  EXPECT_TRUE(
      CompareMatrices(q_expected, q, tolerance_, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(v_expected, v, tolerance_, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(torque_expected, torque, tolerance_,
                              MatrixCompareType::absolute));
}

TEST_F(RobotStateLcmMessageTranslatorTest, TestEncodeDecodeLessJointNames) {
  std::string model = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  // Since this test only affects the non floating base joints, so fixed base
  // is sufficient.
  Initialize(model, true, nullptr,
      FloatingBaseType::kFixed);

  VectorX<double> q, v, torque;

  // Remove a joint from the message.
  message_.num_joints--;
  const int kExtraIndex = 2;
  message_.joint_name.erase(message_.joint_name.begin() + kExtraIndex);
  message_.joint_position.erase(message_.joint_position.begin() + kExtraIndex);
  message_.joint_velocity.erase(message_.joint_velocity.begin() + kExtraIndex);
  message_.joint_effort.erase(message_.joint_effort.begin() + kExtraIndex);

  SetRandomQVTorque(&q, &v, &torque);
  dut_->EncodeMessageKinematics(q, v, &message_);
  dut_->EncodeMessageTorque(torque, &message_);

  EXPECT_EQ(robot_->get_num_positions(), message_.num_joints + 1);
  for (int i = 0; i < robot_->get_num_positions(); ++i) {
    int idx = i;
    if (i == kExtraIndex) continue;

    if (i > kExtraIndex) idx--;

    EXPECT_EQ(robot_->get_position_name(i), message_.joint_name[idx]);
    EXPECT_NEAR(q[i], message_.joint_position[idx], tolerance_);
    EXPECT_NEAR(v[i], message_.joint_velocity[idx], tolerance_);
    EXPECT_NEAR(torque[i], message_.joint_effort[idx], tolerance_);
  }

  // Tests decode, should ignore the second joint.
  VectorX<double> q_expected =
      VectorX<double>::Ones(robot_->get_num_positions());
  VectorX<double> v_expected =
      VectorX<double>::Ones(robot_->get_num_velocities());
  VectorX<double> torque_expected =
      VectorX<double>::Ones(robot_->get_num_actuators());
  dut_->DecodeMessageKinematics(message_, q_expected, v_expected);
  dut_->DecodeMessageTorque(message_, torque_expected);

  for (int i = 0; i < robot_->get_num_positions(); ++i) {
    if (i != kExtraIndex) {
      EXPECT_NEAR(q[i], q_expected[i], tolerance_);
      EXPECT_NEAR(v[i], v_expected[i], tolerance_);
      EXPECT_NEAR(torque[i], torque_expected[i], tolerance_);
    } else {
      EXPECT_EQ(q_expected[i], 1);
      EXPECT_EQ(v_expected[i], 1);
      EXPECT_EQ(torque_expected[i], 1);
    }
  }
}

}  // namespace manipulation
}  // namespace drake
