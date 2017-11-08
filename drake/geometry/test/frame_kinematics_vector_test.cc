#include "drake/geometry/frame_kinematics_vector.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace geometry {
namespace test {

using std::move;
using std::vector;

// Because FrameKinematicsVector inherits from std::vector, we assume that class
// has been properly tested. We limit the testing to the public constructors
// and single additional field.

GTEST_TEST(FrameKinematicsVector, DefaultConstructor) {
  SourceId source_id = SourceId::get_new_id();

  FramePoseVector<double> poses(source_id);

  EXPECT_EQ(poses.get_source_id(), source_id);
  EXPECT_EQ(poses.vector().size(), 0);
  EXPECT_EQ(poses.vector().begin(), poses.vector().end());
}

GTEST_TEST(FrameKinematicsVector, CopyConstructor) {
  SourceId source_id = SourceId::get_new_id();
  std::vector<Isometry3<double>> pose_source;
  int kPoseCount = 5;
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    pose_source.push_back(pose);
  }

  FramePoseVector<double> poses(source_id, pose_source);

  EXPECT_EQ(poses.get_source_id(), source_id);
  EXPECT_EQ(pose_source.size(), kPoseCount);
  EXPECT_EQ(poses.vector().size(), kPoseCount);
  for (int i = 0; i < kPoseCount; ++i) {
    const double kValue = i;
    Vector3<double> pose{kValue, kValue, kValue};
    EXPECT_EQ(poses.mutable_vector()[i].translation(), pose);
  }
}

GTEST_TEST(FrameKinematicsVector, MoveConstructor) {
  SourceId source_id = SourceId::get_new_id();
  std::vector<Isometry3<double>> pose_source;
  int kPoseCount = 5;
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    pose_source.push_back(pose);
  }

  FramePoseVector<double> poses(source_id, move(pose_source));

  EXPECT_EQ(poses.get_source_id(), source_id);
  EXPECT_EQ(pose_source.size(), 0);
  EXPECT_EQ(poses.vector().size(), kPoseCount);
  for (int i = 0; i < kPoseCount; ++i) {
    const double kValue = i;
    Vector3<double> pose{kValue, kValue, kValue};
    EXPECT_EQ(poses.mutable_vector()[i].translation(), pose);
  }
}

GTEST_TEST(FrameKinematicsVector, AutoDiffInstantiation) {
  SourceId source_id = SourceId::get_new_id();
  FramePoseVector<AutoDiffXd> poses(source_id);

  EXPECT_EQ(poses.get_source_id(), source_id);
  EXPECT_EQ(poses.vector().size(), 0);
  EXPECT_EQ(poses.vector().begin(), poses.vector().end());
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
