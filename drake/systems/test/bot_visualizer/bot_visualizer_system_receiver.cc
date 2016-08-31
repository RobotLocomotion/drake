#include "drake/systems/test/bot_visualizer/bot_visualizer_system_receiver.h"

namespace drake {
namespace systems {
namespace test {

BotVisualizerReceiver::BotVisualizerReceiver(::lcm::LCM* lcm,
    std::string channel_postfix) : channel_postfix_(channel_postfix) {
  // Sets up the LCM message subscribers.
  ::lcm::Subscription* sub_load_msg =
      lcm->subscribe("DRAKE_VIEWER_LOAD_ROBOT" + channel_postfix_,
          &BotVisualizerReceiver::HandleLoadMessage, this);
  sub_load_msg->setQueueCapacity(1);

  ::lcm::Subscription* sub_draw_msg =
      lcm->subscribe("DRAKE_VIEWER_DRAW" + channel_postfix_,
          &BotVisualizerReceiver::HandleDrawMessage, this);
  sub_draw_msg->setQueueCapacity(1);

  // Initializes the fields of member variables load_message_ and
  // draw_message_ so the test logic below can determine whether the
  // desired message was received.
  load_message_.num_links = -1;
  draw_message_.num_links = -1;
}

drake::lcmt_viewer_load_robot BotVisualizerReceiver::GetReceivedLoadMessage() {
  drake::lcmt_viewer_load_robot message_copy;

  std::lock_guard<std::mutex> lock(load_message_mutex_);
  message_copy = load_message_;

  return message_copy;
}

drake::lcmt_viewer_draw BotVisualizerReceiver::GetReceivedDrawMessage() {
  drake::lcmt_viewer_draw message_copy;

  std::lock_guard<std::mutex> lock(draw_message_mutex_);
  message_copy = draw_message_;

  return message_copy;
}

void BotVisualizerReceiver::HandleLoadMessage(const ::lcm::ReceiveBuffer* rbuf,
    const std::string& channel_name, const drake::lcmt_viewer_load_robot* msg) {
  if (channel_name == "DRAKE_VIEWER_LOAD_ROBOT" + channel_postfix_) {
    std::lock_guard<std::mutex> lock(load_message_mutex_);
    load_message_ = *msg;
  }
}

void BotVisualizerReceiver::HandleDrawMessage(const ::lcm::ReceiveBuffer* rbuf,
    const std::string& channel_name, const drake::lcmt_viewer_draw* msg) {
  if (channel_name == "DRAKE_VIEWER_DRAW" + channel_postfix_) {
    std::lock_guard<std::mutex> lock(load_message_mutex_);
    draw_message_ = *msg;
  }
}

}  // namespace test
}  // namespace systems
}  // namespace drake
