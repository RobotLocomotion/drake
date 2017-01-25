#include "drake/systems/sensors/rgbd_camera.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

GTEST_TEST(TestRGBDCamera, Instantiate) {
  systems::sensors::RGBDCamera camera;
}
}  // namespace
}  // namespace drake
