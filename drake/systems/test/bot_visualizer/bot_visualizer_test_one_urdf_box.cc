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
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/test/bot_visualizer/bot_visualizer_system_receiver.h"

namespace drake {
namespace systems {
namespace {

const int kDim = 10;
const int kPortNumber = 0;

using drake::parsers::ModelInstanceIdTable;

bool LoadMessageIsGood(const drake::lcmt_viewer_load_robot& load_message,
    const RigidBodyTree& tree, int box_model_instance_id) {
  // Aborts if the load message wasn't received yet.
  if (load_message.num_links == -1) return false;

  // Verifies that the draw message has two links and that their names and
  // model instance IDs are correct.
  int world_model_instance_id =
      tree.get_body(RigidBodyTree::kWorldBodyIndex).get_model_instance_id();

  EXPECT_EQ(load_message.num_links, 2);
  EXPECT_EQ(load_message.link[0].name, "world");
  EXPECT_EQ(load_message.link[0].robot_num, world_model_instance_id);
  EXPECT_EQ(load_message.link[1].name, "box_link");
  EXPECT_EQ(load_message.link[1].robot_num, box_model_instance_id);

  // Verifies that the geometry of the world link is correct. In this case,
  // the world has no geometry.
  EXPECT_EQ(load_message.link[0].num_geom, 0);

  // Verifies that the geometry of the box link is correct. In this case, the
  // box link has a single box geometry.
  EXPECT_EQ(load_message.link[1].num_geom, 1);

  // Note that the following two lines cannot be combined into one due to the
  // following mysterious link error:
  //
  //  gtest-printers.h:276: undefined reference to
  //      `drake::lcmt_viewer_geometry_data::BOX'
  //
  uint8_t box_type = drake::lcmt_viewer_geometry_data::BOX;
  EXPECT_EQ(load_message.link[1].geom[0].type, box_type);

  EXPECT_EQ(load_message.link[1].geom[0].position[0], 0);
  EXPECT_EQ(load_message.link[1].geom[0].position[1], 0);
  EXPECT_EQ(load_message.link[1].geom[0].position[2], 0);

  EXPECT_EQ(load_message.link[1].geom[0].quaternion[0], 1);
  EXPECT_EQ(load_message.link[1].geom[0].quaternion[1], 0);
  EXPECT_EQ(load_message.link[1].geom[0].quaternion[2], 0);
  EXPECT_EQ(load_message.link[1].geom[0].quaternion[3], 0);

  EXPECT_NEAR(load_message.link[1].geom[0].color[0], 0.2, 1e-6);
  EXPECT_NEAR(load_message.link[1].geom[0].color[1], 0.3, 1e-6);
  EXPECT_NEAR(load_message.link[1].geom[0].color[2], 0.4, 1e-6);
  EXPECT_NEAR(load_message.link[1].geom[0].color[3], 0.9, 1e-6);

  EXPECT_EQ(load_message.link[1].geom[0].float_data[0], 1);
  EXPECT_EQ(load_message.link[1].geom[0].float_data[1], 1);
  EXPECT_EQ(load_message.link[1].geom[0].float_data[2], 1);

  return true;
}

bool DrawMessageIsGood(const drake::lcmt_viewer_draw& draw_message,
    const RigidBodyTree& tree, int box_model_instance_id) {
  // Aborts if the draw message wasn't received yet.
  if (draw_message.num_links == -1) return false;

  // Verifies that the draw message has two links and that their names and
  // model instance IDs are correct.
  EXPECT_EQ(draw_message.num_links, 2);
  EXPECT_EQ(draw_message.link_name[0], "world");
  EXPECT_EQ(draw_message.link_name[1], "box_link");
  EXPECT_EQ(draw_message.robot_num[0],
    tree.get_body(RigidBodyTree::kWorldBodyIndex).get_model_instance_id());
  EXPECT_EQ(draw_message.robot_num[1], box_model_instance_id);

  // Verifies that the world link is at position (0, 0, 0).
  EXPECT_EQ(draw_message.position[0][0], 0);
  EXPECT_EQ(draw_message.position[0][1], 0);
  EXPECT_EQ(draw_message.position[0][2], 0);

  // Verifies that the box link is at position (0, 0, 0.5).
  EXPECT_EQ(draw_message.position[1][0], 0);
  EXPECT_EQ(draw_message.position[1][1], 0);
  EXPECT_EQ(draw_message.position[1][2], 0.5);

  // Verifies that the rotations of both the world link and box link are zero.
  EXPECT_EQ(draw_message.quaternion[0][0], 1);
  EXPECT_EQ(draw_message.quaternion[0][1], 0);
  EXPECT_EQ(draw_message.quaternion[0][2], 0);
  EXPECT_EQ(draw_message.quaternion[0][3], 0);
  EXPECT_EQ(draw_message.quaternion[1][0], 1);
  EXPECT_EQ(draw_message.quaternion[1][1], 0);
  EXPECT_EQ(draw_message.quaternion[1][2], 0);
  EXPECT_EQ(draw_message.quaternion[1][3], 0);

  return true;
}

// Tests the functionality of BotVisualizerSystem by making it load box.sdf.
GTEST_TEST(LcmPublisherSystemTest, PublishTest) {
  // Instantiates the LCM subsystem.

  ::lcm::LCM lcm;

  // Instantiates a RigidBodyTree and loads an SDF containing a box into it.
  RigidBodyTree tree;
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::sdf::AddModelInstancesFromSdfFile(
          drake::GetDrakePath() + "/systems/test/bot_visualizer/box.sdf",
          DrakeJoint::FIXED,  // floating_base_type
          nullptr,  // weld_to_frame
          &tree);

  // Verfies that only one model was loaded.
  EXPECT_EQ(model_instance_id_table.size(), 1);

  // Obtains the model instance ID of the box that was just added to the
  // RigidBodyTree.
  int box_model_instance_id = model_instance_id_table.at("box");

  // Instantiates a receiver for the messages that are published by
  // BotVisualizerSystem.
  drake::systems::test::BotVisualizerReceiver receiver(&lcm);

  // Start the LCM recieve thread after all objects it can potentially use
  // are instantiated. Since objects are destructed in the reverse order of
  // construction, this ensures the LCM receive thread stops before any
  // resources it uses are destroyed. If the Lcm receive thread is stopped after
  // the resources it relies on are destroyed, a segmentation fault may occur.
  drake::systems::lcm::LcmReceiveThread lcm_receive_thread(&lcm);

  // Instantiates a BotVisualizerSystem. It is called "dut" to indicate it is
  // the Device Under Test.
  BotVisualizerSystem dut(tree, &lcm);
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
  std::unique_ptr<InputPort<double>> input_port(
      new FreestandingInputPort<double>(std::move(vector_base)));

  context->SetInputPort(kPortNumber, std::move(input_port));

  // Whether the receiver received the LCM messages published by the
  // BotvisualizerSystem.
  bool done = false;

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
    if (LoadMessageIsGood(load_message, tree, box_model_instance_id) &&
        DrawMessageIsGood(draw_message, tree, box_model_instance_id)) {
      done = true;
    }

    if (!done) std::this_thread::sleep_for(std::chrono::milliseconds(kDelayMS));
  }

  EXPECT_TRUE(done);
}

}  // namespace
}  // namespace systems
}  // namespace drake
