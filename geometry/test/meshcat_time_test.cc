#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"

namespace drake {
namespace geometry {
namespace {

// TODO(DamrongGuoy) Remove calls to MaybePauseForUser() when I don't need to
//  test it manually. It's important to invoke it directly with
//  bazel/bin/[PROGRAM].
//     1. bazel build //geometry:meshcat_time_test
//     2. bazel-bin/geometry/meshcat_time_test

// Call SetObject(Shape) with and without time works well together.
GTEST_TEST(MeshcatTet, SetObjectWithShapeTime) {
  Meshcat meshcat;

  // Set untimed object
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3));
  std::cout << "SetObject Box" <<std::endl;
  drake::common::MaybePauseForUser();

  meshcat.StartRecording();
  const MeshcatAnimation& animation = meshcat.get_mutable_recording();
  const double time_start = 0;

  // Set timed objects
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);
  std::cout << "SetObject Cylinder" <<std::endl;
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(animation.frame(time_start),
                                       "morph/<object>", "visible")
                  .value());
  EXPECT_FALSE(
      animation
          .get_key_frame<bool>(animation.frame(t1), "morph/<object>", "visible")
          .value());
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(time_start),
                       fmt::format("morph/<animation>/{}", animation.frame(t1)),
                       "visible")
                   .value());
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(
                      animation.frame(t1),
                      fmt::format("morph/<animation>/{}", animation.frame(t1)),
                      "visible")
                  .value());
  drake::common::MaybePauseForUser();

  const double t2 = 2;
  meshcat.SetObject("morph", Capsule(0.1, 0.3), t2);
  std::cout << "SetObject Capsule" << std::endl;
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(t2),
                       fmt::format("morph/<animation>/{}", animation.frame(t1)),
                       "visible")
                   .value());
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(time_start),
                       fmt::format("morph/<animation>/{}", animation.frame(t2)),
                       "visible")
                   .value());
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(
                      animation.frame(t2),
                      fmt::format("morph/<animation>/{}", animation.frame(t2)),
                      "visible")
                  .value());
  drake::common::MaybePauseForUser();

  const double t3 = 3;
  meshcat.SetObject("morph", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), t3);
  std::cout << "SetObject Ellipsoid" <<std::endl;
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(t3),
                       fmt::format("morph/<animation>/{}", animation.frame(t2)),
                       "visible")
                   .value());
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(time_start),
                       fmt::format("morph/<animation>/{}", animation.frame(t3)),
                       "visible")
                   .value());
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(
                      animation.frame(t3),
                      fmt::format("morph/<animation>/{}", animation.frame(t3)),
                      "visible")
                  .value());
  drake::common::MaybePauseForUser();

  const double time_final = 4;
  meshcat.SetProperty(
      fmt::format("morph/<animation>/{}", animation.frame(t3)),
      "visible", true, time_final);
  std::cout << "SetProperty time_final" << std::endl;
  drake::common::MaybePauseForUser();

  meshcat.StopRecording();
  meshcat.PublishRecording();
  std::cout << "PublishRecording" <<std::endl;
  drake::common::MaybePauseForUser();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
