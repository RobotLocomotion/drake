#include "drake/geometry/geometry_frame.h"

#include <stdexcept>

namespace drake {
namespace geometry {

// N.B. This definition appears in the cc file because get_new_id should never
// be called from a header file.
GeometryFrame::GeometryFrame(const std::string& frame_name, int frame_group_id)
    : id_(FrameId::get_new_id()),
      name_(frame_name),
      frame_group_(frame_group_id) {
  if (frame_group_ < 0) {
    throw std::logic_error(
        "GeometryFrame requires a non-negative frame group");
  }
}

}  // namespace geometry
}  // namespace drake
