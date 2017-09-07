#include "drake/manipulation/util/perception_utils.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_geometry_compare.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace manipulation {
namespace util {
namespace {

using Eigen::Isometry3d;
using Eigen::Quaterniond;

const int kNumOfTrialPoses = 10;

GTEST_TEST(PerceptionUtilsTest, TestVectorIsometry3dConvertors) {
  Isometry3d input_pose_1 = Isometry3d::Identity();
  VectorX<double> expected_pose_1(
      (VectorX<double>(7) << 0, 0, 0, 1, 0, 0, 0).finished());

  EXPECT_TRUE(drake::CompareMatrices(Isometry3dToVector(input_pose_1),
                                     expected_pose_1, 1e-15,
                                     drake::MatrixCompareType::absolute));
  for (int i = 0; i < kNumOfTrialPoses; ++i) {
    Quaterniond pose_quaternion =
        math::QuaternionToCanonicalForm(Quaterniond::UnitRandom());
    Isometry3d input_pose_2;
    input_pose_2.linear() = pose_quaternion.matrix();
    input_pose_2.translation() = VectorX<double>::Random(3);

    VectorX<double> expected_pose = VectorX<double>::Zero(7);
    expected_pose.head<3>() = input_pose_2.translation();
    VectorX<double> expected_quat = VectorX<double>::Zero(4);
    expected_quat << pose_quaternion.w(), pose_quaternion.x(),
        pose_quaternion.y(), pose_quaternion.z();
    expected_pose.tail<4>() = expected_quat;

    EXPECT_TRUE(drake::CompareMatrices(Isometry3dToVector(input_pose_2),
                                       expected_pose, 1e-15,
                                       drake::MatrixCompareType::absolute));
  }
}
GTEST_TEST(PerceptionUtilsTest, TestIsometry3dVectorConvertors) {
  VectorX<double> input_pose_1_vector(
      (VectorX<double>(7) << 0, 0, 0, 1, 0, 0, 0).finished());
  Isometry3d expected_pose_1 = Isometry3d::Identity();

  EXPECT_TRUE(CompareTransforms(VectorToIsometry3d(input_pose_1_vector),
                                expected_pose_1, 1e-15));

  for (int i = 0; i < kNumOfTrialPoses; ++i) {
    Quaterniond pose_quaternion;
    Isometry3d input_pose_2 = Isometry3d::Identity();
    pose_quaternion =
        math::QuaternionToCanonicalForm(Quaterniond::UnitRandom());
    input_pose_2.linear() = pose_quaternion.toRotationMatrix();
    input_pose_2.translation() = VectorX<double>::Random(3);

    Isometry3d return_pose =
        VectorToIsometry3d(Isometry3dToVector(input_pose_2));
    EXPECT_TRUE(CompareTransforms(return_pose, input_pose_2, 1e-10));
  }
}

}  // namespace
}  // namespace util
}  // namespace manipulation
}  // namespace drake
