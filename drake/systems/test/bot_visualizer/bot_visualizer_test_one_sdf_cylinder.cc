#include "drake/systems/bot_visualizer_system.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/test/bot_visualizer/bot_visualizer_system_receiver.h"
#include "drake/systems/test/bot_visualizer/compare_lcm_messages.h"

namespace drake {
namespace systems {
namespace test {
namespace {

const int kPortNumber = 0;

using drake::parsers::ModelInstanceIdTable;

// Tests the functionality of BotVisualizerSystem by making it load
// cylinder.sdf.
GTEST_TEST(LcmPublisherSystemTest, TestOneSdfCylinder) {
  // Defines a channel name postfix to ensure this unit test does not interfere
  // with other unit tests that use BotVisualizerSystem.
  const std::string kChannelPostfix = "_CYLINDER";

  // Instantiates the LCM subsystem.
  ::lcm::LCM lcm;

  // Instantiates a RigidBodyTree and loads an SDF containing a cylinder into
  // it.
  RigidBodyTree tree;
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::sdf::AddModelInstancesFromSdfFile(
          drake::GetDrakePath() + "/systems/test/bot_visualizer/cylinder.sdf",
          DrakeJoint::FIXED,  // floating_base_type
          nullptr,  // weld_to_frame
          &tree);

  // Verfies that only one model was loaded.
  EXPECT_EQ(model_instance_id_table.size(), 1);

  // Obtains the model instance ID of the model that was just added to the
  // RigidBodyTree.
  int model_instance_id = model_instance_id_table.at("cylinder");

  // Instantiates a receiver for the messages that are published by
  // BotVisualizerSystem.
  drake::systems::test::BotVisualizerReceiver receiver(&lcm, kChannelPostfix);

  // Start the LCM recieve thread after all objects it can potentially use
  // are instantiated. Since objects are destructed in the reverse order of
  // construction, this ensures the LCM receive thread stops before any
  // resources it uses are destroyed. If the Lcm receive thread is stopped after
  // the resources it relies on are destroyed, a segmentation fault may occur.
  drake::systems::lcm::LcmReceiveThread lcm_receive_thread(&lcm);

  // Instantiates a BotVisualizerSystem. It is called "dut" to indicate it is
  // the Device Under Test.
  BotVisualizerSystem dut(tree, &lcm, kChannelPostfix);
  EXPECT_EQ(dut.get_name(), "BotVisualizerSystem");

  std::unique_ptr<ContextBase<double>> context = dut.CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

  // Verifies that the context has one input port.
  EXPECT_EQ(context->get_num_input_ports(), 1);

  // Instantiates a BasicVector with known state. This vector holds the joint
  // position states, which the BotVisualizerSystem uses when deriving the
  // transforms that are saved in the published drake::lcmt_viewer_draw
  // messages.
  std::unique_ptr<VectorBase<double>> vector_base(
      new BasicVector<double>(tree.number_of_positions()));

  // Initializes the joint states to be zero.
  {
    Eigen::VectorBlock<VectorX<double>> vector_value =
        vector_base->get_mutable_value();

    vector_value.setZero();
  }

  // Sets the value in the context's input port to be the above-defined
  // VectorInterface. Note that we need to overwrite the original input port
  // created by the BotVisualizerSystem since we do not have write access to its
  // input vector.
  std::unique_ptr<InputPort> input_port(
      new FreestandingInputPort(std::move(vector_base)));

  context->SetInputPort(kPortNumber, std::move(input_port));

  // Whether the receiver received the LCM messages published by the
  // BotvisualizerSystem.
  bool done = false;

  // Defines the expected load and draw messages.
  drake::lcmt_viewer_load_robot expected_load_message;
  {
    expected_load_message.num_links = 2;
    expected_load_message.link.resize(2);
    expected_load_message.link[0].name = std::string(RigidBodyTree::kWorldName);
    expected_load_message.link[0].robot_num =
        tree.get_world_model_instance_id();
    expected_load_message.link[0].num_geom = 0;

    expected_load_message.link[1].name = "cylinder_link";
    expected_load_message.link[1].robot_num = model_instance_id;
    expected_load_message.link[1].num_geom = 1;
    expected_load_message.link[1].geom.resize(1);
    expected_load_message.link[1].geom[0].type =
        drake::lcmt_viewer_geometry_data::CYLINDER;
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
    expected_draw_message.link_name[1] = "cylinder_link";
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
    expected_draw_message.position[1][2] = 1.0;

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

  // This is used to prevent this unit test from running indefinitely when
  // the receiver fails to receive the LCM message published by the
  // BotvisualizerSystem.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  // We must periodically call dut->EvalOutput(...) since we do not know when
  // the receiver will receive the message published by the BotvisualizerSystem.
  while (!done && count++ < kMaxCount) {
    dut.EvalOutput(*context.get(), output.get());

    // Gets the received message.
    const drake::lcmt_viewer_load_robot load_message =
        receiver.GetReceivedLoadMessage();

    const drake::lcmt_viewer_draw draw_message =
        receiver.GetReceivedDrawMessage();

    // Verifies that the size of the received LCM message is correct.
    if (CompareLoadMessage(load_message, expected_load_message) &&
        CompareDrawMessage(draw_message, expected_draw_message)) {
      done = true;
    }

    if (!done) std::this_thread::sleep_for(std::chrono::milliseconds(kDelayMS));
  }

  EXPECT_TRUE(done);
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake
