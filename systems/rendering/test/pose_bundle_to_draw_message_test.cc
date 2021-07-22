#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include <gtest/gtest.h>

#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

GTEST_TEST(PoseBundleToDrawMessageTest, Conversion) {
  PoseBundle<double> bundle(2);
  math::RigidTransformd foo_pose{Eigen::Vector3d{123, 0, 0}};
  bundle.set_name(0, "foo");
  bundle.set_transform(0, foo_pose);
  bundle.set_model_instance_id(0, 42);

  const double roll = -M_PI_2;
  math::RigidTransformd bar_pose{
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()),
      Eigen::Vector3d{0, 0, 0}};
  bundle.set_name(1, "bar");
  bundle.set_transform(1, bar_pose);
  bundle.set_model_instance_id(1, 43);

  PoseBundleToDrawMessage converter;
  auto context = converter.AllocateContext();
  converter.get_input_port(0).FixValue(context.get(), bundle);
  auto output = converter.AllocateOutput();
  converter.CalcOutput(*context, output.get());

  const auto& message = output->get_data(0)->get_value<lcmt_viewer_draw>();
  EXPECT_EQ(2, message.num_links);

  EXPECT_EQ("foo", message.link_name[0]);
  EXPECT_EQ(42, message.robot_num[0]);
  // foo is translated in x.
  EXPECT_EQ(123, message.position[0][0]);  // x
  EXPECT_EQ(0, message.position[0][1]);    // y
  EXPECT_EQ(0, message.position[0][2]);    // z
  // foo has no rotation.
  EXPECT_EQ(1, message.quaternion[0][0]);  // w
  EXPECT_EQ(0, message.quaternion[0][1]);  // x
  EXPECT_EQ(0, message.quaternion[0][2]);  // y
  EXPECT_EQ(0, message.quaternion[0][3]);  // z

  EXPECT_EQ("bar", message.link_name[1]);
  EXPECT_EQ(43, message.robot_num[1]);
  // bar has no translation.
  EXPECT_EQ(0, message.position[1][0]);  // x
  EXPECT_EQ(0, message.position[1][1]);  // y
  EXPECT_EQ(0, message.position[1][2]);  // z
  // bar has a rotation around x.
  EXPECT_NEAR(std::cos(roll / 2), message.quaternion[1][0], 1.0e-6);  // w
  EXPECT_NEAR(std::sin(roll / 2), message.quaternion[1][1], 1.0e-6);  // x
  EXPECT_EQ(0, message.quaternion[0][2]);  // y
  EXPECT_EQ(0, message.quaternion[0][3]);  // z
}

// Tests that PoseBundleToDrawMessageTest allocates no state variables.
GTEST_TEST(PoseBundleToDrawMessageTest, Stateless) {
  PoseBundleToDrawMessage converter;
  auto context = converter.AllocateContext();
  EXPECT_TRUE(context->is_stateless());
}

#pragma GCC diagnostic pop

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
