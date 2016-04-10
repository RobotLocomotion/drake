#include "simple_car.h"

#include "drake/systems/LCMSystem.h"
#include "lcmtypes/drake/lcmt_driving_command_t.hpp"

using namespace std;
using namespace Eigen;
using namespace Drake;
using Drake::getDrakePath;

// N.B.: LCMSystem uses this via c++ ADL magic. See also drake issue #1880.
bool decode(const drake::lcmt_driving_command_t &msg, double &t,
            DrivingCommand<double> &x) {
  t = double(msg.timestamp) / 1000.0;
  x.steering_angle = msg.steering_angle;
  x.throttle = msg.throttle;
  x.brake = msg.brake;
  return true;
}

int main(int argc, const char *argv[]) {
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  auto car = make_shared<Drake::SimpleCar>();
  Drake::SimpleCarState<double> initial_state;
  runLCM(car, lcm, 0, numeric_limits<double>::infinity(), initial_state);

  return 0;
}
