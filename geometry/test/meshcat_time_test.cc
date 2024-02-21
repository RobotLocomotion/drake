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

  meshcat.StartRecording();
  // Set timed object
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), 1);
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<object>"));
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<animation>/64"));

  meshcat.SetObject("morph", Capsule(0.1, 0.3), 2);
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<animation>/128"));

  meshcat.SetObject("morph", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), 3);
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<animation>/192"));

  meshcat.StopRecording();
  meshcat.PublishRecording();

  // TODO(DamrongGuoy) Remove this when I don't need to test it manually.
  meshcat.AddButton("Exit");
  int count_to_exit = 10;
  while (meshcat.GetButtonClicks("Exit")==0 && count_to_exit != 0){
    sleep(1); --count_to_exit;
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
