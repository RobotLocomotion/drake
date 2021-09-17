#include "drake/geometry/internal_frame.h"

#include "drake/common/never_destroyed.h"

namespace drake {
namespace geometry {
namespace internal {

InternalFrame::InternalFrame() {}

InternalFrame::InternalFrame(SourceId source_id, FrameId frame_id,
                             const std::string& name, int frame_group,
                             int index, FrameId parent_id)
    : source_id_(source_id),
      id_(frame_id),
      name_(name),
      frame_group_(frame_group),
      index_(index),
      parent_id_(parent_id) {
  DRAKE_DEMAND(index >= 0);
  DRAKE_DEMAND(frame_group >= 0 || frame_group == world_frame_group());
}

bool InternalFrame::operator==(const InternalFrame& other) const {
  return id_ == other.id_;
}

bool InternalFrame::operator!=(const InternalFrame& other) const {
  return !(*this == other);
}

FrameId InternalFrame::world_frame_id() {
  static const never_destroyed<FrameId> kWorldFrame{FrameId::get_new_id()};
  return kWorldFrame.access();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
