#include "drake/lcm/robotlocomotion_translators/pose_t_translator.h"
#include <gtest/gtest.h>

namespace drake {
namespace lcm {
namespace robotlocomotion_translators {
namespace {

// Tests Isometry3<double> -> robotlocomotion::pose_t.
GTEST_TEST(TranslatorTest, PoseTranslatorEncodeTest) {
  PoseTranslator<double>> dut;

  Isometry3<double> data = Isometry3<double>::Identity();
  data.translation() << 1, 2, 3;
  data.linear() =
      AngleAxis<double>(0.3, Vector3<double>::UnitX()).toRotationMatrix() *
      AngleAxis<double>(-1.0, Vector3<double>::UnitY()).toRotationMatrix() *
      AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()).toRotationMatrix();
  Quaternion<double> expected_quat(data.linear());

  robotlocomotion::pose_t msg;
  dut.Encode(data, &msg);

  EXPECT_EQ(msg.position.x, 1);
  EXPECT_EQ(msg.position.y, 2);
  EXPECT_EQ(msg.position.z, 3);

  Quaternion<double> msg_quat(msg.orientation.w, msg.orientation.x,
                              msg.orientation.y, msg.orientation.z);
  EXPECT_NEAR(std::abs(msg_quat.dot(expected_quat)), 1., 1e-15);
}

// Tests robotlocomotion::pose_t -> Isometry3<double>.
GTEST_TEST(TranslatorTest, PoseTranslatorDecodeTest) {
  PoseTranslator<double>> dut;

  robotlocomotion::pose_t msg;
  msg.position.x = 3;
  msg.position.y = 2;
  msg.position.z = 1;

  Quaternion<double> expected_quat(1, 2, 3, 4);
  expected_quat.normalize();
  msg.orientation.w = expected_quat.w();
  msg.orientation.x = expected_quat.x();
  msg.orientation.y = expected_quat.y();
  msg.orientation.z = expected_quat.z();

  Isometry3<double> data;
  dut.Decode(msg, &data);

  EXPECT_EQ(data.translation()[0], 3);
  EXPECT_EQ(data.translation()[1], 2);
  EXPECT_EQ(data.translation()[2], 1);
  Quaternion<double> quat(data.linear());

  EXPECT_NEAR(std::abs(quat.dot(expected_quat)), 1., 1e-15);
}

}  // namespace
}  // namespace robotlocomotion_translators
}  // namespace lcm
}  // namespace drake
