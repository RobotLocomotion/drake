#include "drake/systems/sensors/rgbd_camera.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

GTEST_TEST(TestRgbdCamera, Instantiate) {
  systems::sensors::RgbdCamera camera;
}
}  // namespace
}  // namespace drake
