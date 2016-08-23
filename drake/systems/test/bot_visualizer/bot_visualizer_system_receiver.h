#pragma once

#include <mutex>

#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"

namespace drake {
namespace systems {
namespace test {

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
 * recently received. This is used for testing BotVisualizerSystem.
 */
class BotVisualizerReceiver {
 public:
  /**
   * A constructor that initializes the subscriptions to the LCM channels on
   * which BotVisualizerSystem publishes.
   */
  explicit BotVisualizerReceiver(::lcm::LCM* lcm);

  /**
   * Returns the most recently received load message.
   */
  drake::lcmt_viewer_load_robot GetReceivedLoadMessage();

  /**
   * Returns the most recently received draw message.
   */
  drake::lcmt_viewer_draw GetReceivedDrawMessage();

 private:
  void HandleLoadMessage(const ::lcm::ReceiveBuffer* rbuf,
                         const std::string& channel_name,
                         const drake::lcmt_viewer_load_robot* msg);

  void HandleDrawMessage(const ::lcm::ReceiveBuffer* rbuf,
                         const std::string& channel_name,
                         const drake::lcmt_viewer_draw* msg);

  std::mutex load_message_mutex_;
  std::mutex draw_message_mutex_;

  drake::lcmt_viewer_load_robot load_message_;
  drake::lcmt_viewer_draw draw_message_;
};

}  // namespace test
}  // namespace systems
}  // namespace drake
