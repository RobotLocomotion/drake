#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/test/bot_visualizer/bot_visualizer_test.h"

namespace drake {
namespace systems {
namespace test {
namespace {

// Tests the functionality of BotVisualizerSystem by making it load
// capsule.sdf.
GTEST_TEST(LcmPublisherSystemTest, TestOneSdfCapsule) {
  // Defines a channel name postfix to ensure this unit test does not interfere
  // with other unit tests that use BotVisualizerSystem.
  std::hash<const char*> hash_function;
  const std::string kChannelPostfix = "_CAPSULE" +
      std::to_string(hash_function(__FILE__));

  // Instantiates the LCM subsystem.
  ::lcm::LCM lcm;

  // Instantiates a RigidBodyTree and loads an SDF containing a capsule into
  // it.
  RigidBodyTree tree;
  drake::parsers::ModelInstanceIdTable model_instance_id_table =
      drake::parsers::sdf::AddModelInstancesFromSdfFile(
          drake::GetDrakePath() + "/systems/test/bot_visualizer/capsule.sdf",
          DrakeJoint::FIXED,  // floating_base_type
          nullptr,  // weld_to_frame
          &tree);

  // Verifies that only one model was loaded.
  EXPECT_EQ(model_instance_id_table.size(), 1);

  // Obtains the model instance ID of the model that was just added to the
  // RigidBodyTree.
  int model_instance_id = model_instance_id_table.at("capsule");

  // Defines the expected load and draw messages.
  drake::lcmt_viewer_load_robot expected_load_message;
  {
    expected_load_message.num_links = 2;
    expected_load_message.link.resize(2);
    expected_load_message.link[0].name = std::string(RigidBodyTree::kWorldName);
    expected_load_message.link[0].robot_num =
        tree.get_world_model_instance_id();
    expected_load_message.link[0].num_geom = 0;

    expected_load_message.link[1].name = "capsule_link";
    expected_load_message.link[1].robot_num = model_instance_id;
    expected_load_message.link[1].num_geom = 1;
    expected_load_message.link[1].geom.resize(1);
    expected_load_message.link[1].geom[0].type =
        drake::lcmt_viewer_geometry_data::CAPSULE;
    expected_load_message.link[1].geom[0].position[0] = 0;
    expected_load_message.link[1].geom[0].position[1] = 0;
    expected_load_message.link[1].geom[0].position[2] = 0;
    expected_load_message.link[1].geom[0].quaternion[0] = 1;
    expected_load_message.link[1].geom[0].quaternion[1] = 0;
    expected_load_message.link[1].geom[0].quaternion[2] = 0;
    expected_load_message.link[1].geom[0].quaternion[3] = 0;
    expected_load_message.link[1].geom[0].color[0] = 0.2;
    expected_load_message.link[1].geom[0].color[1] = 0.3;
    expected_load_message.link[1].geom[0].color[2] = 0.4;
    expected_load_message.link[1].geom[0].color[3] = 0.9;
    expected_load_message.link[1].geom[0].num_float_data = 2;
    expected_load_message.link[1].geom[0].float_data.resize(2);
    expected_load_message.link[1].geom[0].float_data[0] = 1;
    expected_load_message.link[1].geom[0].float_data[1] = 2;
  }

  drake::lcmt_viewer_draw expected_draw_message;
  {
    expected_draw_message.num_links = 2;
    expected_draw_message.link_name.resize(2);
    expected_draw_message.link_name[0] = std::string(RigidBodyTree::kWorldName);
    expected_draw_message.link_name[1] = "capsule_link";
    expected_draw_message.robot_num.resize(2);
    expected_draw_message.robot_num[0] = tree.get_world_model_instance_id();
    expected_draw_message.robot_num[1] = model_instance_id;

    expected_draw_message.position.resize(2);
    expected_draw_message.position[0].resize(3);
    expected_draw_message.position[0][0] = 0;
    expected_draw_message.position[0][1] = 0;
    expected_draw_message.position[0][2] = 0;
    expected_draw_message.position[1].resize(3);
    expected_draw_message.position[1][0] = 0;
    expected_draw_message.position[1][1] = 0;
    expected_draw_message.position[1][2] = 2;

    expected_draw_message.quaternion.resize(2);
    expected_draw_message.quaternion[0].resize(4);
    expected_draw_message.quaternion[0][0] = 1;
    expected_draw_message.quaternion[0][1] = 0;
    expected_draw_message.quaternion[0][2] = 0;
    expected_draw_message.quaternion[0][3] = 0;
    expected_draw_message.quaternion[1].resize(4);
    expected_draw_message.quaternion[1][0] = 1;
    expected_draw_message.quaternion[1][1] = 0;
    expected_draw_message.quaternion[1][2] = 0;
    expected_draw_message.quaternion[1][3] = 0;
  }

  DoBotVisualizerTest(tree, expected_load_message, expected_draw_message,
      kChannelPostfix, &lcm);
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake
