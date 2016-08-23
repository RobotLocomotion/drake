#include "drake/systems/bot_visualizer_system.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
// #include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/basic_vector.h"
// #include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/RigidBodyTree.h"


// #include "drake/systems/lcm/lcm_publisher_system.h"
// #include "drake/systems/lcm/lcm_translator_dictionary.h"
// #include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"


namespace drake {
namespace systems {
namespace {

const int kDim = 10;
const int kPortNumber = 0;

using drake::parsers::ModelInstanceIdTable;

/**
 * Subscribes to the following:
 *
 * <pre>
 * LCM Channel Name         LCM Message Type
 * ----------------         ----------------
 * DRAKE_VIEWER_LOAD_ROBOT  drake::lcmt_viewer_load_robot
 * DRAKE_VIEWER_DRAW        drake::lcmt_viewer_draw
 * </pre>
 *
 * It also provides accessors for obtaining copies of the message that were most
 * recently received.
 */
class BotVisualizerReceiver {
 public:
  BotVisualizerReceiver(::lcm::LCM* lcm) {
    // Sets up the LCM message subscribers.
    ::lcm::Subscription* sub_load_msg =
        lcm->subscribe("DRAKE_VIEWER_LOAD_ROBOT",
            &BotVisualizerReceiver::HandleLoadMessage, this);
    sub_load_msg->setQueueCapacity(1);

    ::lcm::Subscription* sub_draw_msg =
        lcm->subscribe("DRAKE_VIEWER_DRAW",
            &BotVisualizerReceiver::HandleDrawMessage, this);
    sub_draw_msg->setQueueCapacity(1);

    // Initializes the fields of member variables load_message_ and
    // draw_message_ so the test logic below can determine whether the
    // desired message was received.
    load_message_.num_links = -1;
    draw_message_.num_links = -1;
  }

  drake::lcmt_viewer_load_robot GetReceivedLoadMessage() {
    drake::lcmt_viewer_load_robot message_copy;

    std::lock_guard<std::mutex> lock(load_message_mutex_);
    message_copy = load_message_;

    return message_copy;
  }

  drake::lcmt_viewer_draw GetReceivedDrawMessage() {
    drake::lcmt_viewer_draw message_copy;

    std::lock_guard<std::mutex> lock(draw_message_mutex_);
    message_copy = draw_message_;

    return message_copy;
  }

 private:
  void HandleLoadMessage(const ::lcm::ReceiveBuffer* rbuf,
                         const std::string& channel_name,
                         const drake::lcmt_viewer_load_robot* msg) {
    if (channel_name == "DRAKE_VIEWER_LOAD_ROBOT") {
      std::lock_guard<std::mutex> lock(load_message_mutex_);
      load_message_ = *msg;
    }
  }

  void HandleDrawMessage(const ::lcm::ReceiveBuffer* rbuf,
                         const std::string& channel_name,
                         const drake::lcmt_viewer_draw* msg) {
    if (channel_name == "DRAKE_VIEWER_DRAW") {
      std::lock_guard<std::mutex> lock(load_message_mutex_);
      draw_message_ = *msg;
    }
  }

  std::mutex load_message_mutex_;
  std::mutex draw_message_mutex_;

  drake::lcmt_viewer_load_robot load_message_;
  drake::lcmt_viewer_draw draw_message_;
};

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

  // Instantiates a receiver for the messages that are published by
  // BotVisualizerSystem.
  BotVisualizerReceiver receiver(&lcm);

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
  std::unique_ptr<VectorInterface<double>> vector_interface(
      new BasicVector<double>(tree.number_of_positions()));

  // Initializes the joint states to be zero.
  {
    Eigen::VectorBlock<VectorX<double>> vector_value =
        vector_interface->get_mutable_value();

    vector_value.setZero();
  }

  // Sets the value in the context's input port to be the above-defined
  // VectorInterface. Note that we need to overwrite the original input port
  // created by the BotVisualizerSystem since we do not have write access to its
  // input vector.
  std::unique_ptr<InputPort<double>> input_port(
      new FreestandingInputPort<double>(std::move(vector_interface)));

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
    if (load_message.num_links != -1 && draw_message.num_links != -1) {
      done = true;
    }
    // if (received_message.dim == kDim) {
    //   bool values_match = true;

    //   for (int ii = 0; ii < kDim && values_match; ++ii) {
    //     if (received_message.val[ii] != ii) values_match = false;
    //   }

    //   // At this point, if values_match is true, the received message contains
    //   // the expected values, which implies that LcmPublisherSystem successfully
    //   // published the VectorInterface as a drake::lcmt_drake_signal message.
    //   //
    //   // We cannot check whether the following member variables of
    //   // drake::lcmt_drake_signal message was successfully transferred because
    //   // BasicVector does not save this information:
    //   //
    //   //   1. coord
    //   //   2. timestamp
    //   //
    //   // Thus, we must conclude that the experiment succeeded.
    //   if (values_match) done = true;
    // }

    if (!done) std::this_thread::sleep_for(std::chrono::milliseconds(kDelayMS));
  }

  EXPECT_TRUE(done);
}

}  // namespace
}  // namespace systems
}  // namespace drake
