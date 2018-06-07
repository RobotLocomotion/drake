#include "drake/geometry/internal_frame.h"

namespace drake {
namespace geometry {
namespace internal {

const FrameId InternalFrame::kWorldFrame{FrameId::get_new_id()};

InternalFrame::InternalFrame() {}

InternalFrame::InternalFrame(SourceId source_id, FrameId frame_id,
                             const std::string& name, int frame_group,
                             PoseIndex pose_index, FrameId parent_id,
                             int clique)
    : source_id_(source_id),
      id_(frame_id),
      name_(name),
      frame_group_(frame_group),
      pose_index_(pose_index),
      parent_id_(parent_id),
      clique_(clique) {}

bool InternalFrame::operator==(const InternalFrame& other) const {
  return id_ == other.id_;
}

bool InternalFrame::operator!=(const InternalFrame& other) const {
  return !(*this == other);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
