#include "drake/geometry/meshcat_animation.h"

#include <cstdlib>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;

GTEST_TEST(MeshcatAnimationTest, FrameTest) {
  const double kFramesPerSecond = 64;
  MeshcatAnimation animation(kFramesPerSecond);

  EXPECT_EQ(animation.frames_per_second(), kFramesPerSecond);
  EXPECT_EQ(animation.frame(.5), 32);
}

GTEST_TEST(MeshcatAnimationTest, OptionsTest) {
  MeshcatAnimation animation;

  animation.set_autoplay(true);
  EXPECT_TRUE(animation.autoplay());
  animation.set_autoplay(false);
  EXPECT_FALSE(animation.autoplay());

  animation.set_loop_mode(MeshcatAnimation::kLoopOnce);
  EXPECT_EQ(animation.loop_mode(), MeshcatAnimation::kLoopOnce);
  animation.set_loop_mode(MeshcatAnimation::kLoopPingPong);
  EXPECT_EQ(animation.loop_mode(), MeshcatAnimation::kLoopPingPong);

  animation.set_repetitions(14);
  EXPECT_EQ(animation.repetitions(), 14);

  animation.set_clamp_when_finished(true);
  EXPECT_TRUE(animation.clamp_when_finished());
  animation.set_clamp_when_finished(false);
  EXPECT_FALSE(animation.clamp_when_finished());
}

GTEST_TEST(MeshcatAnimationTest, SetTransformTest) {
  MeshcatAnimation animation;
  const int kFrame = 21;

  EXPECT_FALSE(
      animation.get_key_frame<std::vector<double>>(kFrame, "test", "position"));
  EXPECT_FALSE(animation.get_key_frame<std::vector<double>>(kFrame, "test",
                                                            "quaternion"));

  Vector3d position(-2, .5, 8.7);
  Eigen::Quaterniond quaternion(.3, 2, .7, .2);
  quaternion.normalize();
  animation.SetTransform(kFrame, "test", RigidTransformd(quaternion, position));
  std::optional<std::vector<double>> p =
      animation.get_key_frame<std::vector<double>>(kFrame, "test", "position");
  EXPECT_TRUE(p);
  EXPECT_TRUE(CompareMatrices(position, Vector3d::Map(&(*p)[0])));
  std::optional<std::vector<double>> q =
      animation.get_key_frame<std::vector<double>>(kFrame, "test",
                                                   "quaternion");
  EXPECT_TRUE(q);
  Eigen::Quaterniond key_quaternion((*q)[3], (*q)[0], (*q)[1], (*q)[2]);
  EXPECT_TRUE(quaternion.isApprox(key_quaternion, 1e-14));

  EXPECT_EQ(animation.get_javascript_type("test", "position"), "vector3");
  EXPECT_EQ(animation.get_javascript_type("test", "quaternion"), "quaternion");
}

GTEST_TEST(MeshcatAnimationTest, SetPropertyTest) {
  MeshcatAnimation animation;
  const int kFrame = 14;

  // Boolean
  EXPECT_FALSE(animation.get_key_frame<bool>(kFrame, "bool_test", "visible"));
  animation.SetProperty(kFrame, "bool_test", "visible", true);
  std::optional<bool> visible =
      animation.get_key_frame<bool>(kFrame, "bool_test", "visible");
  EXPECT_TRUE(visible);
  EXPECT_TRUE(*visible);
  EXPECT_EQ(animation.get_javascript_type("bool_test", "visible"), "boolean");

  // Double
  EXPECT_FALSE(animation.get_key_frame<double>(kFrame, "double_test",
                                               "material.opacity"));
  animation.SetProperty(kFrame, "double_test", "material.opacity", 0.5);
  std::optional<double> opacity = animation.get_key_frame<double>(
      kFrame, "double_test", "material.opacity");
  EXPECT_TRUE(opacity);
  EXPECT_EQ(*opacity, 0.5);
  EXPECT_EQ(animation.get_javascript_type("double_test", "material.opacity"),
            "number");

  // Double vector
  EXPECT_FALSE(animation.get_key_frame<std::vector<double>>(
      kFrame, "vector_test", "position"));
  animation.SetProperty(kFrame, "vector_test", "position", {0.1, 0.2});
  std::optional<std::vector<double>> vec =
      animation.get_key_frame<std::vector<double>>(kFrame, "vector_test",
                                                   "position");
  EXPECT_TRUE(vec);
  EXPECT_EQ(vec->size(), 2);
  EXPECT_EQ((*vec)[0], .1);
  EXPECT_EQ((*vec)[1], .2);
  EXPECT_EQ(animation.get_javascript_type("vector_test", "position"), "vector");

  // Can't set a different type on a property that's already been set.
  DRAKE_EXPECT_THROWS_MESSAGE(
      animation.SetProperty(kFrame, "bool_test", "visible", 32.0),
      ".*already has a track.*");
  // ... even at a different frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      animation.SetProperty(kFrame + 1, "bool_test", "visible", 32.0),
      ".*already has a track.*");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
