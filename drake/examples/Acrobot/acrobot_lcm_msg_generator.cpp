// Generates simple LCM messages to test acrobot_plant and controllers.

#include <memory>
#include <string>
#include <vector>

#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"
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
  const std::string channel_u = "acrobot_u";
  lcmt_acrobot_x msg_x;
  lcmt_acrobot_u msg_u;
  std::vector<uint8_t> buffer_x(msg_x.getEncodedSize());
  std::vector<uint8_t> buffer_u(msg_u.getEncodedSize());

  MessageHandler handler;
  lcm.Subscribe(channel_x, &handler);
  lcm.StartReceiveThread();

  int t = 0;
  while (t < 1e5) {
    // Publishes msg_x.
    msg_x.theta1 = t * M_PI / 6;
    msg_x.theta2 = t * M_PI / 6;
    msg_x.theta1Dot = std::sin(msg_x.theta1);
    msg_x.theta2Dot = std::cos(msg_x.theta2);

    msg_x.encode(&buffer_x[0], 0, msg_x.getEncodedSize());
    lcm.Publish(channel_x, &buffer_x[0], msg_x.getEncodedSize());

    // Publishes msg_u using received msg_x.
    if (handler.get_receive_channel() == channel_x) {
      // Gets the received message.
      const lcmt_acrobot_x received_msg = handler.GetReceivedMessage();

      // Calculates some output from received state.
      msg_u.tau = received_msg.theta1 + received_msg.theta2;

      // Publish msg_u.
      msg_u.encode(&buffer_u[0], 0, msg_u.getEncodedSize());
      lcm.Publish(channel_u, &buffer_u[0], msg_u.getEncodedSize());
    }
    sleep_for(milliseconds(500));
    t++;
  }

  return 0;
}
}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::acrobot::DoMain(); }
