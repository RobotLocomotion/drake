#include "drake/geometry/meshcat.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/maybe_pause_for_user.h"

namespace drake {
namespace geometry {
namespace {

// Call SetObject(Shape) with and without time works well together.
GTEST_TEST(MeshcatTet, SetObjectWithShapeTime) {
  Meshcat meshcat;

  // Set immediate object
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3));

  meshcat.StartRecording();
  // Set timed objects
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), 1);
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<object>"));
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<animation>/64"));

  meshcat.SetObject("morph", Capsule(0.1, 0.3), 2);
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<animation>/128"));

  meshcat.SetObject("morph", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), 3);
  EXPECT_TRUE(meshcat.HasPath("/drake/morph/<animation>/192"));

  meshcat.StopRecording();
  meshcat.PublishRecording();

  // clang-format off
  // TODO(DamrongGuoy) Remove this when I don't need to test it manually.
  //  It's important to invoke it directly with bazel/bin/[PROGRAM].
  //     bazel build //geometry:meshcat_time_test ; bazel-bin/geometry/meshcat_time_test
  drake::common::MaybePauseForUser();
  // clang-format on
}

}  // namespace
}  // namespace geometry
}  // namespace drake
