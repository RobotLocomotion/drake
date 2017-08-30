#include "drake/manipulation/util/perception_utils.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_geometry_compare.h"

namespace drake {
namespace manipulation {
namespace util {
namespace {

using Eigen::Isometry3d;
using Eigen::Quaterniond;

GTEST_TEST(PerceptionUtilsTest, TestVectorIsometry3dConvertors) {
Isometry3d pose_1 = Isometry3d::Identity();
VectorX<double> expected_pose_1((VectorX<double>(7) << 0, 0, 0, 1, 0, 0, 0).finished());

EXPECT_TRUE(drake::CompareMatrices(Isometry3dToVector(pose_1),
                                   expected_pose_1, 1e-15,
                                   drake::MatrixCompareType::absolute));
 for(int i = 0; i < 5; ++i) {
  Quaterniond pose_quaternion = math::QuaternionToCanonicalForm(Quaterniond::UnitRandom());
  Isometry3d pose_2;
  pose_2.linear() = pose_quaternion.matrix();
  pose_2.translation() = VectorX<double>::Random(3);

  VectorX<double> expected_pose = VectorX<double>::Zero(7);
  expected_pose.head<3>() = pose_2.translation();
  VectorX<double> expected_quat = VectorX<double>::Zero(4);
  expected_quat<<pose_quaternion.w(), pose_quaternion.x(),
    pose_quaternion.y(), pose_quaternion.z();
  expected_pose.tail<4>() = expected_quat;

//drake::log()->info("\nTest Position {},\n TestQuat {}", pose_2.translation().transpose(), expected_quat.transpose() );
  EXPECT_TRUE(drake::CompareMatrices(Isometry3dToVector(pose_2),
                                     expected_pose,
                                     1e-15,
                                   drake::MatrixCompareType::absolute));

}

}
GTEST_TEST(PerceptionUtilsTest, TestRandomPose) {

 for (int i = 0; i<10; ++i) {

  Quaterniond pose_quaternion = math::QuaternionToCanonicalForm(Quaterniond::UnitRandom());
  Isometry3d input_pose = Isometry3d::Identity();
  input_pose.linear() = pose_quaternion.toRotationMatrix();
  input_pose.translation() = VectorX<double>::Random(3);

  VectorX<double> expected_pose = VectorX<double>::Zero(7);
  expected_pose.head<3>() = input_pose.translation();
  VectorX<double> expected_quat = VectorX<double>::Zero(4);
  expected_quat<<pose_quaternion.w(), pose_quaternion.x(),
      pose_quaternion.y(), pose_quaternion.z();
  expected_pose.tail<4>() = expected_quat;

  Isometry3d return_pose = VectorToIsometry3d(Isometry3dToVector(input_pose));
  EXPECT_TRUE(
      CompareTransforms(return_pose,
                        input_pose, 1e-10));
 }
}


//
//
//Eigen::Isometry3d VectorToIsometry3d(const VectorX<double> &pose_vector);
//
//VectorX<double> Isometry3dToVector(const Eigen::Isometry3d& pose);
//
//}

}  // namespace
}  // namespace util
}  // namespace manipulation
}  // namespace drake