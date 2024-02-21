#include "drake/geometry/meshcat.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

// Call SetObject(Shape) with and without time works well together.
GTEST_TEST(MeshcatTet, SetObjectWithShapeTime) {
  Meshcat meshcat;

  // Set untimed object
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3));
  EXPECT_TRUE(meshcat.HasPath("morph"));
  EXPECT_TRUE(meshcat.HasPath("/drake/morph"));

  // I'm not sure yet why I don't have this path. It prevents me from
  // checking that the visible property is turned on and off at the right time.
  // For now, I have to do manual test:
  //     bazel run //geometry:meshcat_time_test
  //     Click on Meshcat URL in console's message.
  // which is unreliable. Sometimes I had to run twice.
  EXPECT_FALSE(meshcat.HasPath("/drake/morph/<object>"));

  meshcat.StartRecording();
  // Set timed object
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), 1);
  meshcat.SetObject("morph", Capsule(0.1, 0.3), 2);
  meshcat.SetObject("morph", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), 3);

  meshcat.StopRecording();
  meshcat.PublishRecording();

  // TODO(DamrongGuoy) Remove sleep() when I don't need to test it manually.
  sleep(5);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
