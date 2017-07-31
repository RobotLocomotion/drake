#include "drake/geometry/internal_frame.h"

namespace drake {
namespace geometry {
namespace internal {

const FrameId InternalFrame::kWorldFrame{FrameId::get_new_id()};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
