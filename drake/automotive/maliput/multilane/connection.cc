#include "drake/automotive/maliput/multilane/connection.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {

std::ostream& operator<<(std::ostream& out, const EndpointXy& endpoint_xy) {
  return out << "(x = " << endpoint_xy.x() << ", y = " << endpoint_xy.y()
             << ", heading = " << endpoint_xy.heading() << ")";
}

std::ostream& operator<<(std::ostream& out, const EndpointZ& endpoint_z) {
  return out << "(z = " << endpoint_z.z() << ", z_dot = " << endpoint_z.z_dot()
             << ", theta = " << endpoint_z.theta()
             << ", theta_dot = " << endpoint_z.theta_dot() << ")";
}

std::ostream& operator<<(std::ostream& out, const Endpoint& endpoint) {
  return out << "(xy: " << endpoint.xy() << ", z: " << endpoint.z() << ")";
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
