#include "drake/geometry/dev/internal_frame.h"

namespace drake {
namespace geometry {
namespace dev {
namespace internal {

InternalFrame::InternalFrame() {}

InternalFrame::InternalFrame(SourceId source_id, FrameId frame_id,
                             const std::string& name, int frame_group,
                             InternalIndex internal_index, FrameId parent_id,
                             int clique)
    : source_id_(source_id),
      id_(frame_id),
      name_(name),
      frame_group_(frame_group),
      internal_index_(internal_index),
      clique_(clique),
      parent_id_(parent_id) {}

bool InternalFrame::operator==(const InternalFrame& other) const {
  return id_ == other.id_;
}

bool InternalFrame::operator!=(const InternalFrame& other) const {
  return !(*this == other);
}

}  // namespace internal
}  // namespace dev
}  // namespace geometry
}  // namespace drake
