// TODO(jwnimmer-tri) Replace this with a simple command-line argument
// to steering_command_driver.py.

#include <iostream>
#include <sstream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

using namespace std;

int main(int argc, char** argv) {
  if (argc < 3)
    cout << "Usage: publishDrivingCommand throttle steering_angle"
         << endl;

  lcm::LCM lcm;
  if (!lcm.good()) return 1;

  drake::lcmt_driving_command_t command;
  command.timestamp = 0;

  stringstream t(argv[1]);
  t >> command.throttle;
  command.brake = 0.0;

  stringstream s(argv[2]);
  s >> command.steering_angle;

  lcm.publish("DRIVING_COMMAND", &command);

  return 0;
}
