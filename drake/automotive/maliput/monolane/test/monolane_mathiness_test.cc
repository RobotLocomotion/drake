#include "drake/automotive/maliput/monolane/arc_lane.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/automotive/maliput/monolane/lane.h"
#include "drake/automotive/maliput/monolane/line_lane.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/segment.h"

#include <cmath>
#include <iostream>

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace monolane {

GTEST_TEST(MonolaneMathinessTest, Rot3) {
  Rot3 yaw90 {M_PI / 2., 0., 0.};
  EXPECT_NEAR(yaw90.apply({1., 0., 0.}).x, 0., 1e-6);
  EXPECT_NEAR(yaw90.apply({1., 0., 0.}).y, 1., 1e-6);
  EXPECT_NEAR(yaw90.apply({1., 0., 0.}).z, 0., 1e-6);

  EXPECT_NEAR(yaw90.apply({0., 1., 0.}).x, -1., 1e-6);
  EXPECT_NEAR(yaw90.apply({0., 1., 0.}).y,  0., 1e-6);
  EXPECT_NEAR(yaw90.apply({0., 1., 0.}).z,  0., 1e-6);
}

}
}
}
