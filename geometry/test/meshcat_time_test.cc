#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"

namespace drake {
namespace geometry {
namespace {

// TODO(DamrongGuoy) Remove the following manual test. For now, invoke it
//  directly with bazel/bin/[PROGRAM].
//     1. bazel build //geometry:meshcat_time_test
//     2. bazel-bin/geometry/meshcat_time_test

GTEST_TEST(MeshcatTet, ManualTestSetObjectWithShapeTime) {
  Meshcat meshcat;

  // Set untimed object
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3));
  std::cout << "SetObject Box (untimed)" << std::endl;
  drake::common::MaybePauseForUser();

  meshcat.StartRecording();
  const MeshcatAnimation& animation = meshcat.get_mutable_recording();

  // Set timed objects
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);
  std::cout << "SetObject Cylinder at time=" << t1 << std::endl;
  drake::common::MaybePauseForUser();

  const double t2 = 2;
  meshcat.SetObject("morph", Capsule(0.1, 0.3), t2);
  std::cout << "SetObject Capsule at time=" << t2 << std::endl;
  drake::common::MaybePauseForUser();

  const double t3 = 3;
  meshcat.SetObject("morph", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), t3);
  std::cout << "SetObject Ellipsoid at time=" << t3 << std::endl;
  drake::common::MaybePauseForUser();

  const double time_final = 4;
  meshcat.SetProperty(fmt::format("morph/<animation>/{}", animation.frame(t3)),
                      "visible", true, time_final);
  std::cout << "SetProperty time_final=" << time_final << std::endl;
  drake::common::MaybePauseForUser();

  meshcat.StopRecording();
  meshcat.PublishRecording();
  std::cout << "PublishRecording" << std::endl;
  drake::common::MaybePauseForUser();
}

// Set timed object
GTEST_TEST(MeshcatTet, SetObjectWithShapeTime0) {
  Meshcat meshcat;

  meshcat.StartRecording();
  const double time_start = 0;

  // Set timed objects
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);

  const MeshcatAnimation& animation = meshcat.get_mutable_recording();
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(time_start),
                       fmt::format("morph/<animation>/{}", animation.frame(t1)),
                       "visible")
                   .value_or(true));
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(
                      animation.frame(t1),
                      fmt::format("morph/<animation>/{}", animation.frame(t1)),
                      "visible")
                  .value_or(false));
}

// Set untimed object then timed object
GTEST_TEST(MeshcatTet, SetObjectWithShapeTime1) {
  Meshcat meshcat;

  // Set untimed object
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3));

  meshcat.StartRecording();
  const double time_start = 0;

  // Set timed objects
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);

  const MeshcatAnimation& animation = meshcat.get_mutable_recording();
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(animation.frame(time_start),
                                       "morph/<object>", "visible")
                  .value_or(false));
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(time_start),
                       fmt::format("morph/<animation>/{}", animation.frame(t1)),
                       "visible")
                   .value_or(true));
  EXPECT_FALSE(
      animation
          .get_key_frame<bool>(animation.frame(t1), "morph/<object>", "visible")
          .value_or(true));
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(
                      animation.frame(t1),
                      fmt::format("morph/<animation>/{}", animation.frame(t1)),
                      "visible")
                  .value_or(false));
}

// Set two consecutive timed objects
GTEST_TEST(MeshcatTet, SetObjectWithShapeTime2) {
  Meshcat meshcat;
  meshcat.StartRecording();
  const double time_start = 0;

  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);

  const double t2 = 2;
  meshcat.SetObject("morph", Capsule(0.1, 0.3), t2);

  const MeshcatAnimation& animation = meshcat.get_mutable_recording();
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(time_start),
                       fmt::format("morph/<animation>/{}", animation.frame(t2)),
                       "visible")
                   .value_or(true));
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(t2),
                       fmt::format("morph/<animation>/{}", animation.frame(t1)),
                       "visible")
                   .value_or(true));
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(
                      animation.frame(t2),
                      fmt::format("morph/<animation>/{}", animation.frame(t2)),
                      "visible")
                  .value_or(false));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
