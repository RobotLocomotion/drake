// Generates simple LCM messages to test acrobot_plant and controllers.

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_acrobot_u.hpp"
#include "drake/lcmt_acrobot_x.hpp"
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

  // Decode channel_x into msg_x.
  drake::lcm::Subscriber<lcmt_acrobot_x> subscription(&lcm, channel_x);
  const lcmt_acrobot_x& msg_x = subscription.message();

  for (int i = 0; i < 1e5; ++i) {
    // Publishes a dummy msg_x.
    {
      lcmt_acrobot_x dummy{};
      dummy.theta1 = i * M_PI / 6;
      dummy.theta2 = i * M_PI / 6;
      dummy.theta1Dot = std::sin(dummy.theta1);
      dummy.theta2Dot = std::cos(dummy.theta2);
      Publish(&lcm, channel_x, dummy);
    }

    // Publishes msg_u using received msg_x.
    lcm.HandleSubscriptions(0 /* timeout */);
    if (msg_x.timestamp > 0) {
      // Calculates some output from received state.
      lcmt_acrobot_u msg_u{};
      msg_u.tau = msg_x.theta1 + msg_x.theta2;
      Publish(&lcm, channel_u, msg_u);
    }

    sleep_for(milliseconds(500));
  }

  return 0;
}
}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::acrobot::DoMain(); }
