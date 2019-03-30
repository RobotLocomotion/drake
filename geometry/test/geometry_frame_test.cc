#include "drake/geometry/geometry_frame.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

using std::make_unique;
using std::move;

GTEST_TEST(GeometryFrameTest, Constructor) {
  // Case: use default frame group.
  {
    GeometryFrame frame("frame");
    EXPECT_EQ(frame.frame_group(), 0);
  }

  // Case: user-specified, valid frame group.
  {
    GeometryFrame frame("frame", 17);
    EXPECT_EQ(frame.frame_group(), 17);
  }

  // Case: user-specified, invalid frame group.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        GeometryFrame("name", -1),
        std::logic_error,
        "GeometryFrame requires a non-negative frame group");
  }
}

GTEST_TEST(GeometryFrameTest, IdCopies) {
  GeometryFrame frame_a{"frame_a"};
  GeometryFrame frame_b{frame_a};
  EXPECT_EQ(frame_a.id(), frame_b.id());

  GeometryFrame frame_c{"frame_c"};
  EXPECT_NE(frame_c.id(), frame_a.id());
  frame_c = frame_a;
  EXPECT_EQ(frame_c.id(), frame_a.id());
}

}  // namespace
}  // namespace geometry
}  // namespace drake
