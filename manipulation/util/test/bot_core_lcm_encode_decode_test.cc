#include "drake/manipulation/util/bot_core_lcm_encode_decode.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"

using Eigen::Sequential;
using Eigen::Vector3d;

namespace drake {
namespace {

// TODO(jwnimmer-tri) Unit tests should not use unseeded randomness.

GTEST_TEST(TestLcmUtil, testVector3d) {
  const int kVectorSize = 3;
  const Eigen::Vector3d vec = Vector3d::LinSpaced(Sequential, 0, kVectorSize);
  bot_core::vector_3d_t msg;
  EncodeVector3d(vec, msg);
  Eigen::Vector3d vec_back = DecodeVector3d(msg);
  EXPECT_TRUE(CompareMatrices(vec, vec_back, 0.0, MatrixCompareType::absolute));
}

GTEST_TEST(TestLcmUtil, testQuaternion) {
  std::default_random_engine generator;
  generator.seed(0);
  const Eigen::Quaterniond q = math::UniformlyRandomQuaternion(&generator);
  bot_core::quaternion_t msg;
  EncodeQuaternion(q, msg);
  auto quat_back = DecodeQuaternion(msg);
  EXPECT_TRUE(CompareMatrices(Eigen::Vector4d(q.w(), q.x(), q.y(), q.z()),
                              quat_back, 0.0, MatrixCompareType::absolute));
}

GTEST_TEST(TestLcmUtil, testPose) {
  std::default_random_engine generator;
  generator.seed(0);
  const math::RigidTransform<double> pose(
    math::UniformlyRandomRotationMatrix(&generator),
    Vector3d(0, 1, 2));
  bot_core::position_3d_t msg;
  EncodePose(pose.GetAsIsometry3(), msg);
  const math::RigidTransform<double> pose_back(DecodePose(msg));
  EXPECT_TRUE(CompareMatrices(pose.GetAsMatrix34(), pose_back.GetAsMatrix34(),
                              1E-12, MatrixCompareType::absolute));
}

GTEST_TEST(TestLcmUtil, testTwist) {
  const drake::TwistVector<double> twist =
      drake::TwistVector<double>::LinSpaced(Sequential, 0, drake::kTwistSize);
  bot_core::twist_t msg;
  EncodeTwist(twist, msg);
  auto twist_back = DecodeTwist(msg);
  EXPECT_TRUE(
      CompareMatrices(twist, twist_back, 0.0, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace drake

