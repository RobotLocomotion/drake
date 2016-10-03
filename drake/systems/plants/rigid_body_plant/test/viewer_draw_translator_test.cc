#include "drake/systems/plants/rigid_body_plant/viewer_draw_translator.h"

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/RollPitchYawFloatingJoint.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;

using drake::math::rpy2quat;

// Tests the basic functionality of the translator.
GTEST_TEST(ViewerDrawTranslatorTests, BasicTest) {
  // Creates a `RigidBodyTree` with `kNumBodies` rigid bodies.
  const int kNumBodies = 2;

  auto tree = make_unique<RigidBodyTree>();
  for (int i = 0; i < kNumBodies; ++i) {
    auto body = make_unique<RigidBody>();
    body->set_name("body" + std::to_string(i));
    body->set_model_instance_id(tree->add_model_instance());

    // The inertia model must be set to prevent RigidBodyTree::compile() from
    // replacing the body's joint with a FixedJoint.
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "Joint" + std::to_string(i), Eigen::Isometry3d::Identity());

    body->add_joint(&tree->world(), std::move(joint));

    tree->bodies.push_back(std::move(body));
  }
  tree->compile();

  // Creates an `LcmtViewerDrawTranslator` object using the `RigidBodyTree` that
  // was just created. This is the "device under test."
  ViewerDrawTranslator viewer_draw_translator(*tree);

  // Instantiates a generalized state vector containing all zeros. This is
  // selected to make the resulting quaternion values be easily specified.
  int num_states = tree->get_num_positions() + tree->get_num_velocities();
  BasicVector<double> generalized_state(num_states);
  generalized_state.set_value(Eigen::VectorXd::Zero(num_states));

  // Places body0 to be at location X = 1, Y = 2, Z = 3 with an orientation of
  // roll = PI.
  generalized_state.SetAtIndex(0, 1);
  generalized_state.SetAtIndex(1, 2);
  generalized_state.SetAtIndex(2, 3);
  generalized_state.SetAtIndex(3, M_PI);

  // Uses the `ViewerDrawTranslator` to convert the `BasicVector<double>` into
  // a byte array for a `drake::lcmt_viewer_draw` message.
  double time = 0;
  std::vector<uint8_t> message_bytes;
  viewer_draw_translator.Serialize(time, generalized_state, &message_bytes);
  EXPECT_GT(message_bytes.size(), 0u);

  // Verifies that the serialized message is correct. This entails:
  //     (1) manually creating a the correct `drake::lcmt_viewer_draw`
  //     (2) serializing it into an array of bytes
  //     (3) verifying that the byte array matches `message_bytes`

  // TODO(liang.fok): Replace the following two lines with
  // `Eigen::Quaterniond::Identity()` and a method in lcmUtil.h that converts it
  // into a std::vector<float>. Related issue: #3470.
  std::vector<float> zero_position = {0, 0, 0};
  std::vector<float> zero_quaternion = {1, 0, 0, 0};

  std::vector<float> body0_position = {1, 2, 3};

  Vector3<double> body0_rpy = {M_PI, 0, 0};
  Vector4<double> body0_quaternion_eigen = rpy2quat(body0_rpy);
  std::vector<float> body0_quaternion(4);
  body0_quaternion[0] = static_cast<float>(body0_quaternion_eigen(0));
  body0_quaternion[1] = static_cast<float>(body0_quaternion_eigen(1));
  body0_quaternion[2] = static_cast<float>(body0_quaternion_eigen(2));
  body0_quaternion[3] = static_cast<float>(body0_quaternion_eigen(3));

  lcmt_viewer_draw expected_message;
  expected_message.timestamp = static_cast<int64_t>(time * 1000);
  expected_message.num_links = tree->get_num_bodies();
  expected_message.link_name.push_back("world");
  expected_message.link_name.push_back("body0");
  expected_message.link_name.push_back("body1");
  expected_message.robot_num.push_back(0);
  expected_message.robot_num.push_back(0);
  expected_message.robot_num.push_back(1);
  expected_message.position.push_back(zero_position);
  expected_message.position.push_back(body0_position);
  expected_message.position.push_back(zero_position);
  expected_message.quaternion.push_back(zero_quaternion);
  expected_message.quaternion.push_back(body0_quaternion);
  expected_message.quaternion.push_back(zero_quaternion);

  const int byte_count = expected_message.getEncodedSize();
  EXPECT_EQ(byte_count, static_cast<int>(message_bytes.size()));

  std::vector<uint8_t> expected_bytes(byte_count);
  expected_message.encode(expected_bytes.data(), 0, byte_count);

  EXPECT_EQ(expected_bytes, message_bytes);
}

}  // namespace
}  // namespace systems
}  // namespace drake
