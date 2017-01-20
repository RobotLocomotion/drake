// generates simple LCM messages to test acrobot_plant_w_lcm

#include <memory>

#include "drake/examples/acrobot/lcmt_acrobot_u.hpp"
#include "drake/examples/acrobot/lcmt_acrobot_x.hpp"

#include "drake/examples/Acrobot/acrobot_lcm_msg_handler.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace acrobot {
namespace {

using std::chrono::milliseconds;
using std::this_thread::sleep_for;

int DoMain() {
  drake::lcm::DrakeLcm lcm;

  const std::string channel_x = "acrobot_xhat";
  lcmt_acrobot_x msg_x;
  std::vector<uint8_t> buffer(msg_x.getEncodedSize());

  MessageHandler handler;
  lcm.Subscribe(channel_x, &handler);
  lcm.StartReceiveThread();

  int t = 0;
  while (t <= 1000) {
    // publishing messages to be received
    msg_x.theta1 = t * M_PI / 6;
    msg_x.theta2 = t * M_PI / 6;
    msg_x.theta1Dot = std::sin(msg_x.theta1);
    msg_x.theta2Dot = std::cos(msg_x.theta2);

    msg_x.encode(&buffer[0], 0, msg_x.getEncodedSize());
    lcm.Publish(channel_x, &buffer[0], msg_x.getEncodedSize());

    // receive lcm messages
    if (handler.get_receive_channel() == channel_x) {
      // Gets the received message.
      const lcmt_acrobot_x received_msg = handler.GetReceivedMessage();

      std::cout << "theta1 = " << received_msg.theta1
                << " ,theta2 = " << received_msg.theta2
                << " ,theta1Dot = " << received_msg.theta1Dot
                << " ,theta2Dot = " << received_msg.theta2Dot << std::endl;
    }
    sleep_for(milliseconds(500));
    t++;
  }

  // publish tau
  /*
  const std::string channel_u = "acrobot_u";
  lcmt_acrobot_u msg_u;
  std::vector<uint8_t> buffer(msg_u.getEncodedSize());

  int t = 0;
  while (t <= 1000) {
    // publishing messages to be received
    msg_u.tau = 2*(t%2)-1;

    msg_u.encode(&buffer[0], 0, msg_u.getEncodedSize());
    lcm.Publish(channel_u, &buffer[0], msg_u.getEncodedSize());

    sleep_for(milliseconds(1000));
    t++;
  }
   */
  return 0;
}
}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::acrobot::DoMain(); }
