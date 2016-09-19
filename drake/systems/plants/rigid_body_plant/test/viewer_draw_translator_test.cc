#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/RollPitchYawFloatingJoint.h"
#include "drake/systems/plants/rigid_body_plant/viewer_draw_translator.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;

// Tests the basic functionality of the translator.
GTEST_TEST(ViewerDrawTranslatorTests, BasicTest) {
  // Define the number of rigid bodies to add to the `RigidbodyTree`.
  const int kNumBodies = 2;

  // Creates a `RigidBodyTree` with `kNumBodies` rigid bodies.
  auto tree = make_unique<RigidBodyTree>();
  for (int i = 0; i < kNumBodies; ++i) {
    auto body = make_unique<RigidBody>();
    body->set_name("body" + std::to_string(i));
    body->set_model_instance_id(tree->add_model_instance());

    // The inertia model must be set to prevent RigidBodyTree::compile() from
    // replacing the body's joint with a FixedJoint.
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    // Connects the body to the world using a RPY floating joint.
    auto joint = make_unique<RollPitchYawFloatingJoint>(
        "Joint" + std::to_string(i), Eigen::Isometry3d::Identity());

    body->add_joint(&tree->world(), std::move(joint));

    // Adds the body to the tree.
    tree->bodies.push_back(std::move(body));
  }
  tree->compile();

  // Creates an `LcmtViewerDrawTranslator` object using the tree that was just
  // created. The name "dut" stands for "Device Under Test".
  ViewerDrawTranslator dut(*tree.get());

  // Instantiates a `BasicVector<double>`. Since there are `kNumBodies` bodies,
  // there are `kNumBodies * ViewerDrawTranslator::kNumStatesPerBody` values.
  int num_states = tree->number_of_positions() + tree->number_of_velocities();

  BasicVector<double> generalized_state(num_states);
  for (int i = 0; i < num_states; ++i) {
    generalized_state.SetAtIndex(i, i);
  }

  // Transforms the basic vector into the byte array representation of a
  // `drake::lcmt_viewer_draw` message.
  double time = 0;
  std::vector<uint8_t> message_bytes;
  dut.Serialize(time, generalized_state, &message_bytes);
  EXPECT_GT(message_bytes.size(), 0);

  // Verifies that the serialized message is correct. This entails:
  //     (1) manually creating a the correct `drake::lcmt_viewer_draw`
  //     (2) serializing it into an array of bytes
  //     (3) verifying that the byte array matches `message_bytes`
  lcmt_viewer_draw expected_message;
  expected_message.timestamp = time;
  expected_message.num_links = tree->get_number_of_bodies();
  const Eigen::VectorXd q = generalized_state.CopyToVector().head(
      tree->number_of_positions());
  KinematicsCache<double> cache = tree->doKinematics(q);
  for (int i = 0; i < expected_message.num_links; ++i) {
    const RigidBody& body = tree->get_body(i);
    expected_message.link_name.push_back(body.get_name());
    expected_message.robot_num.push_back(body.get_model_instance_id());

    auto transform = tree->relativeTransform(cache, 0, i);
    auto quat = drake::math::rotmat2quat(transform.linear());

    std::vector<float> position;
    auto translation = transform.translation();
    for (int j = 0; j < 3; ++j) {
      position.push_back(static_cast<float>(translation(j)));
    }
    expected_message.position.push_back(position);

    std::vector<float> orientation;
    for (int j = 0; j < 4; ++j) {
      orientation.push_back(static_cast<float>(quat(j)));
    }
    expected_message.quaternion.push_back(orientation);
  }

  const int expected_message_length = expected_message.getEncodedSize();
  EXPECT_EQ(expected_message_length, message_bytes.size());

  std::vector<uint8_t> expected_message_bytes;
  expected_message_bytes.resize(expected_message_length);
  expected_message.encode(expected_message_bytes.data(), 0,
                          expected_message_length);

  for (int i = 0; i < expected_message_length; ++i) {
    EXPECT_EQ(expected_message_bytes[i], message_bytes[i]);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
