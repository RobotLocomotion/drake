#include "drake/examples/Cars/car.h"


bool decode(const drake::lcmt_driving_control_cmd_t& msg, double& t,
            DrivingCommand<double>& x) {
  t = double(msg.timestamp) / 1000.0;
  x.steering_angle = msg.steering_angle;
  x.throttle = msg.throttle_value;
  x.brake = msg.brake_value;
  return true;
}


namespace Drake {


}
