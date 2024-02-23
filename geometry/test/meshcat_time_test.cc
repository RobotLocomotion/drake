#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"

namespace drake {
namespace geometry {
namespace {

bool HasVisibleProperty(Meshcat* meshcat, double time,
                                  const std::string& path) {
  MeshcatAnimation& animation = meshcat->get_mutable_recording();
  return animation.get_key_frame<bool>(animation.frame(time), path, "visible")
      .has_value();
}

bool GetVisibleProperty(Meshcat* meshcat, double time,
                        const std::string& path) {
  MeshcatAnimation& animation = meshcat->get_mutable_recording();
  return *animation.get_key_frame<bool>(animation.frame(time), path, "visible");
}

int frame(Meshcat* meshcat, double time) {
  MeshcatAnimation& animation = meshcat->get_mutable_recording();
  return animation.frame(time);
}

// Call SetObject(Shape) with and without time works well together.
GTEST_TEST(MeshcatTet, SetObjectWithShapeTime) {
  Meshcat meshcat;

  // Set untimed object
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3));

  // TODO(DamrongGuoy) Remove this when I don't need to test it manually.
  //  It's important to invoke it directly with bazel/bin/[PROGRAM].
  //     1. bazel build //geometry:meshcat_time_test
  //     2. bazel-bin/geometry/meshcat_time_test
  drake::common::MaybePauseForUser();

  meshcat.StartRecording();

  // Set timed objects
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);

  ASSERT_TRUE(meshcat.HasPath("morph/<object>"));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t1, "morph/<object>"));
  EXPECT_FALSE(GetVisibleProperty(&meshcat, t1, "morph/<object>"));

  const std::string path_t1 =
      fmt::format("morph/<animation>/{}", frame(&meshcat, t1));

  ASSERT_TRUE(meshcat.HasPath(path_t1));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t1, path_t1));
  EXPECT_TRUE(GetVisibleProperty(&meshcat, t1, path_t1));

  const double t2 = 2;
  meshcat.SetObject("morph", Capsule(0.1, 0.3), t2);

  ASSERT_TRUE(meshcat.HasPath(path_t1));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t2, path_t1));
  EXPECT_FALSE(GetVisibleProperty(&meshcat, t2, path_t1));

  const std::string path_t2 =
      fmt::format("morph/<animation>/{}", frame(&meshcat, t2));

  ASSERT_TRUE(meshcat.HasPath(path_t2));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t2, path_t2));
  EXPECT_TRUE(GetVisibleProperty(&meshcat, t2, path_t2));

  const double t3 = 3;
  meshcat.SetObject("morph", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), t3);

  ASSERT_TRUE(meshcat.HasPath(path_t2));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t3, path_t2));
  EXPECT_FALSE(GetVisibleProperty(&meshcat, t3, path_t2));

  const std::string path_t3 =
      fmt::format("morph/<animation>/{}", frame(&meshcat, t3));

  ASSERT_TRUE(meshcat.HasPath(path_t3));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t3, path_t3));
  EXPECT_TRUE(GetVisibleProperty(&meshcat, t3, path_t3));

  meshcat.StopRecording();
  meshcat.PublishRecording();

  // TODO(DamrongGuoy) Remove this when I don't need to test it manually.
  drake::common::MaybePauseForUser();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
