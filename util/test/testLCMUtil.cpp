/* clang-format off to disable clang-format-includes */
#include "drake/util/lcmUtil.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/random_rotation.h"

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
  const auto quaternion = drake::math::UniformlyRandomQuat(generator);
  bot_core::quaternion_t msg;
  EncodeQuaternion(quaternion, msg);
  auto quat_back = DecodeQuaternion(msg);
  EXPECT_TRUE(
      CompareMatrices(quaternion, quat_back, 0.0, MatrixCompareType::absolute));
}

GTEST_TEST(TestLcmUtil, testPose) {
  std::default_random_engine generator;
  generator.seed(0);
  Eigen::Isometry3d pose;
  pose.linear() = drake::math::UniformlyRandomRotmat(generator);
  pose.translation().setLinSpaced(0, drake::kSpaceDimension);
  pose.makeAffine();
  const Eigen::Isometry3d& const_pose = pose;
  bot_core::position_3d_t msg;
  EncodePose(const_pose, msg);
  Eigen::Isometry3d pose_back = DecodePose(msg);
  EXPECT_TRUE(CompareMatrices(pose.matrix(), pose_back.matrix(), 1e-12,
                              MatrixCompareType::absolute));
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

