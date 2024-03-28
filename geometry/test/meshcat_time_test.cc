#include <fmt/format.h>
#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_types_internal.h"

namespace drake {
namespace geometry {
namespace {

// TODO(DamrongGuoy) Remove the following manual test. For now, invoke it
//  directly with bazel/bin/[PROGRAM].
//     1. bazel build //geometry:meshcat_time_test
//     2. bazel-bin/geometry/meshcat_time_test

GTEST_TEST(MeshcatTest, ManualTestSetObjectWithShapeTime) {
  Meshcat meshcat;

  // Set the unanimated object
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3), Rgba(.9, .9, .9, .5));
  std::cout << "SetObject Box without time" << std::endl;
  drake::common::MaybePauseForUser();

  meshcat.StartRecording();

  // Set the animated object at three time_in_recording = t1, t2, t3
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1, Rgba(.9, .9, .9, .5));
  std::cout << "SetObject Cylinder at time=" << t1 << std::endl;
  drake::common::MaybePauseForUser();
  const double t2 = 2;
  meshcat.SetObject("morph", Capsule(0.1, 0.3), t2, Rgba(.9, .9, .9, .5));
  std::cout << "SetObject Capsule at time=" << t2 << std::endl;
  drake::common::MaybePauseForUser();
  const double t3 = 3;
  meshcat.SetObject("morph", Ellipsoid(0.1, 0.1, (0.3 + 0.1 + 0.1) / 2), t3,
                    Rgba(.9, .9, .9, .5));
  std::cout << "SetObject Ellipsoid at time=" << t3 << std::endl;
  drake::common::MaybePauseForUser();

  meshcat.StopRecording();
  meshcat.PublishRecording();
  std::cout << "PublishRecording" << std::endl;
  drake::common::MaybePauseForUser();

  // After PublishRecording(), set the unanimated object again as a sphere.
  // This caused a side effect that playing animation in the web browser (Drake
  // MeshCat tab) will play the animated objects while keeping the unanimated
  // sphere on.
  // For now, the easiest way to move forward is probably to prohibit this case.
  // See https://github.com/RobotLocomotion/drake/issues/21139
  meshcat.SetObject("morph", Sphere(0.25), Rgba(.9, .9, .9, .5));
  std::cout << "SetObject Sphere without time" << std::endl;
  drake::common::MaybePauseForUser();
}

GTEST_TEST(MeshcatTest, ThrowIfAnimateObjectBeforeRecording) {
  Meshcat meshcat;
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.SetObject("path", Sphere(1), /*time_in_recording*/ 0.5),
      "You must create a recording.*before calling SetObject with time");
}

GTEST_TEST(MeshcatTest, ThrowIfAnimateObjectAfterStopRecoding) {
  Meshcat meshcat;
  meshcat.StartRecording();
  meshcat.SetObject("path", Sphere(1.1), /*time_in_recording*/ 0.1);
  meshcat.StopRecording();
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.SetObject("path", Sphere(1.2), /*time_in_recording*/ 0.2),
      "You must not stop recording.*before calling SetObject with time");
}

GTEST_TEST(MeshcatTest, ThrowIfAnimateObjectOfEarlierFrame) {
  Meshcat meshcat;
  meshcat.StartRecording();
  meshcat.SetObject("path", Sphere(1.2), /*time_in_recording*/ 0.2);
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.SetObject("path", Sphere(1.1), /*time_in_recording*/ 0.1),
      "SetObject with time_in_recording.*to an earlier frame.*");
}

GTEST_TEST(MeshcatTest, AnimateObject) {
  Meshcat meshcat;
  meshcat.StartRecording();
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);

  const MeshcatAnimation& animation = meshcat.get_mutable_recording();
  EXPECT_FALSE(meshcat
                   .GetPackedObject(
                       fmt::format("morph/<animation>/{}", animation.frame(t1)))
                   .empty());
  // In the recorded animation, it's invisible at the start time and
  // becomes visible at the set time t1.
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(animation.start_time()),
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

GTEST_TEST(MeshcatTest, UnanimateThenAnimate) {
  Meshcat meshcat;
  meshcat.SetObject("morph", Box(0.2, 0.2, 0.3));
  meshcat.StartRecording();
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);

  const MeshcatAnimation& animation = meshcat.get_mutable_recording();
  // In the recorded animation, the untimed object is visible at the start
  // time and becomes invisible at the animation time t1.
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(animation.frame(animation.start_time()),
                                       "morph/<object>", "visible")
                  .value_or(false));
  EXPECT_FALSE(
      animation
          .get_key_frame<bool>(animation.frame(t1), "morph/<object>", "visible")
          .value_or(true));
  // In the recorded animation, the animated object is invisible at the start
  // time and becomes visible at the animation time t1.
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(animation.start_time()),
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

GTEST_TEST(MeshcatTest, AnimateThenAnimate) {
  Meshcat meshcat;
  meshcat.StartRecording();
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);
  const double t2 = 2;
  meshcat.SetObject("morph", Capsule(0.1, 0.3), t2);

  const MeshcatAnimation& animation = meshcat.get_mutable_recording();
  // In the recorded animation, the first timed object becomes invisible at
  // the second recorded time t2.
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(t2),
                       fmt::format("morph/<animation>/{}", animation.frame(t1)),
                       "visible")
                   .value_or(true));
  // In the recorded animation, the second timed object is invisible at
  // the start time and becomes visible at the second recorded time t2.
  EXPECT_FALSE(animation
                   .get_key_frame<bool>(
                       animation.frame(animation.start_time()),
                       fmt::format("morph/<animation>/{}", animation.frame(t2)),
                       "visible")
                   .value_or(true));
  EXPECT_TRUE(animation
                  .get_key_frame<bool>(
                      animation.frame(t2),
                      fmt::format("morph/<animation>/{}", animation.frame(t2)),
                      "visible")
                  .value_or(false));
}

// We should prohibit this case.  See 21139.
GTEST_TEST(MeshcatTest, AnimateThenUnanimate) {
  Meshcat meshcat;
  meshcat.StartRecording();
  const double t1 = 1;
  meshcat.SetObject("morph", Cylinder(0.1, 0.3), t1);
  meshcat.SetObject("morph", Sphere(0.25));

  // The timed object becomes invisible.
  std::string property = meshcat.GetPackedProperty(
      fmt::format("morph/<animation>/{}",
                  meshcat.get_mutable_recording().frame(t1)),
      "visible");
  ASSERT_FALSE(property.empty());
  EXPECT_FALSE(msgpack::unpack(property.data(), property.size())
                   .get()
                   .as<internal::SetPropertyData<bool>>()
                   .value);
  // The delay in the websocket thread made it hard to verify that the
  // untimed object is visible right away. Here we trust that it was tested
  // elsewhere.
}

}  // namespace
}  // namespace geometry
}  // namespace drake
