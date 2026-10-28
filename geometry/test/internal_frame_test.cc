#include "drake/geometry/internal_frame.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

// The world frame is anchored by construction; any other frame starts dynamic
// and becomes anchored only when told. Anchoring is sticky.
GTEST_TEST(InternalFrameTest, Anchoring) {
  const SourceId source_id = SourceId::get_new_id();

  const InternalFrame world(source_id, InternalFrame::world_frame_id(), "world",
                            InternalFrame::world_frame_group(), 0,
                            InternalFrame::world_frame_id());
  EXPECT_TRUE(world.is_anchored());

  InternalFrame frame(source_id, FrameId::get_new_id(), "frame", 0, 1,
                      InternalFrame::world_frame_id());
  EXPECT_FALSE(frame.is_anchored());
  frame.set_anchored();
  EXPECT_TRUE(frame.is_anchored());
  frame.set_anchored();
  EXPECT_TRUE(frame.is_anchored());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
