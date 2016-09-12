#include "drake/systems/test/bot_visualizer/bot_visualizer_test.h"

#include "gtest/gtest.h"

#include "drake/systems/bot_visualizer_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/test/bot_visualizer/bot_visualizer_system_receiver.h"
#include "drake/systems/test/bot_visualizer/compare_lcm_messages.h"

namespace drake {
namespace systems {
namespace test {

const int kPortNumber = 0;


void DoBotVisualizerTest(const RigidBodyTree& tree,
    const drake::lcmt_viewer_load_robot& expected_load_message,
    const drake::lcmt_viewer_draw& expected_draw_message,
    const std::string& kChannelPostfix,
    ::lcm::LCM* lcm) {
  // Instantiates a receiver for the messages that are published by
  // BotVisualizerSystem.
  drake::systems::test::BotVisualizerReceiver receiver(lcm, kChannelPostfix);

  // Start the LCM receive thread after all objects it can potentially use
  // are instantiated. Since objects are destructed in the reverse order of
  // construction, this ensures the LCM receive thread stops before any
  // resources it uses are destroyed. If the LCM receive thread is stopped after
  // the resources it relies on are destroyed, a segmentation fault may occur.
  drake::systems::lcm::LcmReceiveThread lcm_receive_thread(lcm);

  // Instantiates a BotVisualizerSystem. It is called "dut" to indicate it is
  // the Device Under Test.
  BotVisualizerSystem dut(tree, lcm, kChannelPostfix);
  EXPECT_EQ(dut.get_name(), "BotVisualizerSystem");

  std::unique_ptr<ContextBase<double>> context = dut.CreateDefaultContext();

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

  // This is used to prevent this unit test from running indefinitely when
  // the receiver fails to receive the LCM message published by the
  // BotvisualizerSystem.
  int count = 0;

  // Defines the maximum number of times this unit test attempts to have
  // BotVisualizerSystem sends the draw commands.
  const int kMaxCount = 10;

  // Defines the number of milliseconds to wait between transmissions.
  const int kDelayMS = 500;

  // We must periodically call dut->EvalOutput(...) since we do not know when
  // the receiver will receive the message published by the BotvisualizerSystem.
  while (!done && count++ < kMaxCount) {
    dut.Publish(*context.get());

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

}  // namespace test
}  // namespace systems
}  // namespace drake
