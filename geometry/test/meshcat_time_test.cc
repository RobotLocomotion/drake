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

// Verify the transpilation algorithm without SetObject(...time...) yet.
GTEST_TEST(MeshcatTet, TranspileManually) {
  Meshcat meshcat;

  // Set untimed object
  meshcat.SetObject("foo", Box(0.2, 0.2, 0.3), Rgba(.9, .9, .9, 0.5));
  drake::common::MaybePauseForUser();

  const double time_start = 0;

  meshcat.StartRecording(
      // double frames_per_second = 64.0,
      // bool set_visualizations_while_recording = true
      );
  meshcat.SetProperty("foo/<object>", "visible", true, time_start);

  const double t1 = 1;
  const int frame_t1 = meshcat.get_mutable_recording().frame(t1);

  // Transpile SetObject("foo", Cylinder(0.1, 0.3), t1)
  meshcat.SetProperty("foo/<object>", "visible", false, t1);
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t1),
                      "visible", false, time_start);
  meshcat.SetObject(fmt::format("foo/<animation>/{}", frame_t1),
                    Cylinder(0.1, 0.3), Rgba(.9, .9, .9, 0.5));
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t1),
                      "visible", true, t1);
  drake::common::MaybePauseForUser();

  const double t2 = 2;
  const int frame_t2 = meshcat.get_mutable_recording().frame(t2);

  // Transpile SetObject("foo", Capsule(0.1, 0.3), t2)
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t1),
                      "visible", false, t2);
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t2),
                      "visible", false, time_start);
  meshcat.SetObject(fmt::format("foo/<animation>/{}", frame_t2),
                    Capsule(0.1, 0.3), Rgba(.9, .9, .9, 0.5));
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t2),
                      "visible", true, t2);
  drake::common::MaybePauseForUser();

  const double t3 = 3;
  const int frame_t3 = meshcat.get_mutable_recording().frame(t3);

  // Transpile SetObject("foo", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), t3)
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t2),
                      "visible", false, t3);
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t3),
                      "visible", false, time_start);
  meshcat.SetObject(fmt::format("foo/<animation>/{}", frame_t3),
                    Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2),
                    Rgba(.9, .9, .9, 0.5));
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t3),
                      "visible", true, t3);
  drake::common::MaybePauseForUser();

  const double time_final = 4;
  meshcat.SetProperty(fmt::format("foo/<animation>/{}", frame_t3),
                      "visible", true, time_final);

  meshcat.StopRecording();
  meshcat.PublishRecording();
  std::cout << "PublishRecording" <<std::endl;
  drake::common::MaybePauseForUser();
}


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
  std::cout << "SetObject Box" <<std::endl;
  drake::common::MaybePauseForUser();

  meshcat.StartRecording();

  // Set timed objects
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);
  std::cout << "SetObject Cylinder" <<std::endl;
  drake::common::MaybePauseForUser();

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
  std::cout << "SetObject Capsule" <<std::endl;
  drake::common::MaybePauseForUser();

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
  std::cout << "SetObject Ellipsoid" <<std::endl;
  drake::common::MaybePauseForUser();

  ASSERT_TRUE(meshcat.HasPath(path_t2));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t3, path_t2));
  EXPECT_FALSE(GetVisibleProperty(&meshcat, t3, path_t2));

  const std::string path_t3 =
      fmt::format("morph/<animation>/{}", frame(&meshcat, t3));

  ASSERT_TRUE(meshcat.HasPath(path_t3));
  ASSERT_TRUE(HasVisibleProperty(&meshcat, t3, path_t3));
  EXPECT_TRUE(GetVisibleProperty(&meshcat, t3, path_t3));

  const double time_final = 4;
  meshcat.SetProperty(fmt::format("morph/<animation>/{}", frame(&meshcat, t3)),
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
