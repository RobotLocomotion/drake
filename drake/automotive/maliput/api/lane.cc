#include "drake/automotive/maliput/api/lane.h"

#include <iostream>
#include <string>

namespace drake {
namespace maliput {
namespace api {

std::ostream& operator<<(std::ostream& out, const LaneId& lane_id) {
  return out << std::string("Lane(") << lane_id.id << std::string(")");
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
