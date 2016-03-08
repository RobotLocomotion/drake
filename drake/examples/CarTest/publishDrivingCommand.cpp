
#include <iostream>
#include <sstream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drake/lcmt_driving_control_cmd_t.hpp"

using namespace std;

int main(int argc, char** argv) {
  if (argc < 3)
    cout << "Usage: publishDrivingCommand throttle_value steering_value"
         << endl;

  lcm::LCM lcm;
  if (!lcm.good()) return 1;

  drake::lcmt_driving_control_cmd_t command;
  command.timestamp = 0;

  stringstream t(argv[1]);
  t >> command.throttle_value;

  stringstream s(argv[2]);
  s >> command.steering_angle;

  command.brake_value = 0.0;
  lcm.publish("DRIVING_COMMAND", &command);

  return 0;
}
