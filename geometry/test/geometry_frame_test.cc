#include "drake/geometry/geometry_frame.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

using std::make_unique;
using std::move;

GTEST_TEST(GeometryFrameTest, IdCopies) {
  Isometry3<double> pose = Isometry3<double>::Identity();
  GeometryFrame frame_a{"frame_a", pose};
  GeometryFrame frame_b{frame_a};
  EXPECT_EQ(frame_a.id(), frame_b.id());

  GeometryFrame frame_c{"frame_c", pose};
  EXPECT_NE(frame_c.id(), frame_a.id());
  frame_c = frame_a;
  EXPECT_EQ(frame_c.id(), frame_a.id());
}

}  // namespace
}  // namespace geometry
}  // namespace drake
