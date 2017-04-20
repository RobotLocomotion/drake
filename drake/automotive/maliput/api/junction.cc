#include "drake/automotive/maliput/api/junction.h"

#include <iostream>
#include <string>

namespace drake {
namespace maliput {
namespace api {

std::ostream& operator<<(std::ostream& out, const JunctionId& junction_id) {
  return out << std::string("Junction(") << junction_id.id << std::string(")");
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
