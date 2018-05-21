#include "drake/automotive/maliput/multilane/builder_spec.h"

namespace drake {
namespace maliput {
namespace multilane {

std::ostream& operator<<(std::ostream& out,
                         const StartReference::Spec& start_spec) {
  return out << "(endpoint: " << start_spec.endpoint() << ")";
}

std::ostream& operator<<(std::ostream& out,
                         const EndReference::Spec& end_spec) {
  return out << "(endpoint_z: " << end_spec.endpoint_z() << ")";
}

std::ostream& operator<<(std::ostream& out, const LaneLayout& lane_layout) {
  return out << "(left_shoulder: " << lane_layout.left_shoulder()
             << ", right_shoulder: " << lane_layout.right_shoulder()
             << ", num_lanes: " << lane_layout.num_lanes()
             << ", ref_lane: " << lane_layout.ref_lane()
             << ", ref_r0: " << lane_layout.ref_r0() << ")";
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
