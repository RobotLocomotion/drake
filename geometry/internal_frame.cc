#include "drake/geometry/internal_frame.h"

namespace drake {
namespace geometry {
namespace internal {

const FrameId InternalFrame::kWorldFrame{FrameId::get_new_id()};

InternalFrame::InternalFrame() {}

InternalFrame::InternalFrame(SourceId source_id, FrameId frame_id,
                             const std::string& name, int frame_group,
                             FrameIndex index, FrameId parent_id, int clique)
    : source_id_(source_id),
      id_(frame_id),
      name_(name),
      frame_group_(frame_group),
      index_(index),
      clique_(clique),
      parent_id_(parent_id) {
  DRAKE_DEMAND(frame_group >= 0 || frame_group == world_frame_group());
  DRAKE_DEMAND(clique >= 0 || clique == world_frame_clique());
}

bool InternalFrame::operator==(const InternalFrame& other) const {
  return id_ == other.id_;
}

bool InternalFrame::operator!=(const InternalFrame& other) const {
  return !(*this == other);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
