#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/test/bot_visualizer/bot_visualizer_test.h"

namespace drake {
namespace systems {
namespace test {
namespace {

// Tests the functionality of BotVisualizerSystem by making it load a URDF
// that references a mesh.
GTEST_TEST(LcmPublisherSystemTest, TestOneUrdfMesh) {
  // Defines a channel name postfix to ensure this unit test does not interfere
  // with other unit tests that use BotVisualizerSystem.
  std::hash<const char*> hash_function;
  const std::string kChannelPostfix = "_MESH" +
      std::to_string(hash_function(__FILE__));

  // Instantiates the LCM subsystem.
  ::lcm::LCM lcm;

  // Instantiates a RigidBodyTree and loads an URDF containing a mesh into it.
  RigidBodyTree tree;
  drake::parsers::ModelInstanceIdTable model_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() +
              "/examples/Quadrotor/crazyflie/crazyflie.urdf",
          DrakeJoint::FIXED,  // floating_base_type
          nullptr,  // weld_to_frame
          &tree);

  // Verifies that only one model was loaded.
  EXPECT_EQ(model_instance_id_table.size(), 1);

  // Obtains the model instance ID of the model that was just added to the
  // RigidBodyTree.
  int model_instance_id = model_instance_id_table.at("crazyflie");


  // Defines the expected load and draw messages.
  drake::lcmt_viewer_load_robot expected_load_message;
  {
    expected_load_message.num_links = 2;
    expected_load_message.link.resize(2);
    expected_load_message.link[0].name = std::string(RigidBodyTree::kWorldName);
    expected_load_message.link[0].robot_num =
        tree.get_world_model_instance_id();
    expected_load_message.link[0].num_geom = 0;

    expected_load_message.link[1].name = "base_link";
    expected_load_message.link[1].robot_num = model_instance_id;
    expected_load_message.link[1].num_geom = 1;
    expected_load_message.link[1].geom.resize(1);
    expected_load_message.link[1].geom[0].type =
        drake::lcmt_viewer_geometry_data::MESH;
    expected_load_message.link[1].geom[0].position[0] = 0;
    expected_load_message.link[1].geom[0].position[1] = 0;
    expected_load_message.link[1].geom[0].position[2] = -0.01;
    expected_load_message.link[1].geom[0].quaternion[0] = -0.27059805;
    expected_load_message.link[1].geom[0].quaternion[1] = -0.27059805;
    expected_load_message.link[1].geom[0].quaternion[2] = 0.6532815;
    expected_load_message.link[1].geom[0].quaternion[3] = 0.6532815;
    expected_load_message.link[1].geom[0].color[0] = 0;
    expected_load_message.link[1].geom[0].color[1] = 0;
    expected_load_message.link[1].geom[0].color[2] = 1;
    expected_load_message.link[1].geom[0].color[3] = 0;

    // The following string is not the full string, but rather just an expected
    // substring of the real string. The appropriate logic is implemented in
    // CompareLoadMessage(), see compare_lcm_messages.h.
    expected_load_message.link[1].geom[0].string_data =
        "drake/examples/Quadrotor/crazyflie/mesh/crazyflie.obj";
    expected_load_message.link[1].geom[0].num_float_data = 3;
    expected_load_message.link[1].geom[0].float_data.resize(3);
    expected_load_message.link[1].geom[0].float_data[0] = 0.001;
    expected_load_message.link[1].geom[0].float_data[1] = 0.001;
    expected_load_message.link[1].geom[0].float_data[2] = 0.001;
  }

  drake::lcmt_viewer_draw expected_draw_message;
  {
    expected_draw_message.num_links = 2;
    expected_draw_message.link_name.resize(2);
    expected_draw_message.link_name[0] = std::string(RigidBodyTree::kWorldName);
    expected_draw_message.link_name[1] = "base_link";
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
    expected_draw_message.position[1][2] = 0;

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
