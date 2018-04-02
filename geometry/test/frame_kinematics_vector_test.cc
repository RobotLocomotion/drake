#include "drake/geometry/frame_kinematics_vector.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace test {


GTEST_TEST(FrameKinematicsVector, Constructor) {
  SourceId source_id = SourceId::get_new_id();

  FramePoseVector<double> poses1(source_id, 0);

  EXPECT_EQ(poses1.source_id(), source_id);
  EXPECT_EQ(poses1.size(), 0);
  EXPECT_EQ(poses1.count(), 0);

  const int kCount = 5;
  FramePoseVector<double> poses2(source_id, kCount);

  EXPECT_EQ(poses2.source_id(), source_id);
  EXPECT_EQ(poses2.size(), kCount);
  EXPECT_EQ(poses2.count(), 0);
}

GTEST_TEST(FrameKinematicsVector, SettingValues) {
  SourceId source_id = SourceId::get_new_id();
  int kPoseCount = 3;
  FramePoseVector<double> poses(source_id, kPoseCount);
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    poses.set_value(FrameId::get_new_id(), pose);
  }

  EXPECT_EQ(poses.count(), kPoseCount);

  DRAKE_EXPECT_THROWS_MESSAGE(
      poses.set_value(FrameId::get_new_id(), Isometry3<double>::Identity()),
      std::runtime_error,
      "Trying to report kinematics data for more frames "
      "than the vector was sized for; did you forget to call clear.*")
}

GTEST_TEST(FrameKinematicsVector, Clear) {
  SourceId source_id = SourceId::get_new_id();
  int kPoseCount = 3;
  FramePoseVector<double> poses(source_id, kPoseCount);
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    poses.set_value(FrameId::get_new_id(), pose);
  }

  EXPECT_EQ(poses.count(), kPoseCount);
  poses.clear();

  // Clearing the vector leaves the size unchanged and resets the count.
  EXPECT_EQ(poses.count(), 0);
  EXPECT_EQ(poses.size(), kPoseCount);
}

GTEST_TEST(FrameKinematicsVector, AutoDiffInstantiation) {
  SourceId source_id = SourceId::get_new_id();
  const int kCount = 2;
  FramePoseVector<AutoDiffXd> poses(source_id, kCount);

  EXPECT_EQ(poses.source_id(), source_id);
  EXPECT_EQ(poses.size(), kCount);
  EXPECT_EQ(poses.count(), 0);
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
