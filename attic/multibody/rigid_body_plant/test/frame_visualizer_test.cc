#include "drake/multibody/rigid_body_plant/frame_visualizer.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(FrameVisualizerTests, TestMessageGeneration) {
  RigidBodyTree<double> tree;

  const char* kModelPath =
      "drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf = FindResourceOrThrow(kModelPath);
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kFixed, &tree);

  // Visualizes the 7th body's frame.
  std::vector<RigidBodyFrame<double>> local_transforms;
  Isometry3<double> X_BF = Isometry3<double>::Identity();
  X_BF.linear() =
      AngleAxis<double>(0.3, Vector3<double>::UnitZ()).toRotationMatrix();
  X_BF.translation() = Vector3<double>(1, 2, 3);

  local_transforms.push_back(
      RigidBodyFrame<double>("iiwa_link_ee",
          tree.FindBody("iiwa_link_ee"), X_BF));

  drake::lcm::DrakeMockLcm lcm;

  for (int i = 0; i < 2; ++i) {
    drake::log()->debug("i: {}", i);

    systems::FrameVisualizer dut(&tree, local_transforms, &lcm);
    std::string lcm_channel = "DRAKE_DRAW_FRAMES";
    if (i == 1) {
      lcm_channel = "DRAKE_DRAW_FRAMES_CUSTOM";
      dut.set_lcm_channel(lcm_channel);
    }
    drake::lcm::Subscriber<drake::lcmt_viewer_draw> draw_sub(
        &lcm, lcm_channel);

    auto context = dut.CreateDefaultContext();
    EXPECT_EQ(1, context->num_input_ports());

    // Initializes the system's input vector to contain all zeros.
    const int vector_size =
        tree.get_num_positions() + tree.get_num_velocities();
    auto input_data =
        std::make_unique<systems::BasicVector<double>>(vector_size);
    VectorX<double> x = VectorX<double>::Zero(vector_size);
    input_data->set_value(x);
    context->FixInputPort(0, std::move(input_data));
    dut.Publish(*context);

    KinematicsCache<double> cache = tree.CreateKinematicsCache();
    auto q = x.head(tree.get_num_positions());
    auto v = x.head(tree.get_num_velocities());
    cache.initialize(q, v);
    tree.doKinematics(cache);
    const Isometry3<double> X_WF = tree.CalcFramePoseInWorldFrame(
        cache, local_transforms[0]);
    const Quaternion<double> quat_tmp(X_WF.linear());
    // Casts to float, because the message is in float.
    const Quaternion<float> q_WF = quat_tmp.cast<float>();
    const Vector3<float> p_WF = X_WF.translation().cast<float>();

    drake::lcmt_viewer_draw expected_message{};
    expected_message.timestamp = context->get_time();
    expected_message.num_links = 1;
    expected_message.link_name.push_back("iiwa_link_ee");
    expected_message.robot_num.push_back(0);
    expected_message.position.push_back({p_WF[0], p_WF[1], p_WF[2]});
    expected_message.quaternion.push_back(
        {q_WF.w(), q_WF.x(), q_WF.y(), q_WF.z()});

    // Serialize the expected message.
    std::vector<uint8_t> expected_bytes(expected_message.getEncodedSize());
    expected_message.encode(expected_bytes.data(), 0, expected_bytes.size());

    // Serialize the actual message.
    lcm.HandleSubscriptions(0);
    const auto& message = draw_sub.message();
    std::vector<uint8_t> actual_bytes(message.getEncodedSize());
    message.encode(actual_bytes.data(), 0, actual_bytes.size());

    // Verifies that the messages are equal.
    EXPECT_EQ(expected_bytes, actual_bytes);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
