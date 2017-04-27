#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"

namespace drake {
namespace examples {
namespace acrobot {

/// Handles received LCM messages of type lcmt_acrobot_x.
// copied from drake/lcm/test/drake_lcm_test.cc
class MessageHandler : public lcm::DrakeLcmMessageHandlerInterface {
 public:
  /// A constructor that initializes the memory for storing received LCM
  /// messages.
  MessageHandler() {
    // Initializes the fields of received_message.
    received_message_.theta1 = 0;
    received_message_.theta2 = 0;
    received_message_.theta1Dot = 0;
    received_message_.theta2Dot = 0;
    received_message_.timestamp = 0;
  }

  /// This is the callback method.
  void HandleMessage(const std::string& channel, const void* message_buffer,
                     int message_size) override {
    channel_ = channel;
    std::lock_guard<std::mutex> lock(message_mutex_);
    received_message_.decode(message_buffer, 0, message_size);
  }

  /// Returns a copy of the most recently received message.
  lcmt_acrobot_x GetReceivedMessage() {
    lcmt_acrobot_x message_copy;
    std::lock_guard<std::mutex> lock(message_mutex_);
    message_copy = received_message_;
    return message_copy;
  }

  /// Returns the channel on which the most recent message was received.
  const std::string& get_receive_channel() { return channel_; }

 private:
  std::string channel_{};
  std::mutex message_mutex_;
  lcmt_acrobot_x received_message_;
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
