#include "drake/geometry/frame_kinematics_vector.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace test {

GTEST_TEST(FrameKinematicsVector, Constructor) {
  SourceId source_id = SourceId::get_new_id();

  std::vector<FrameId> ids;
  FramePoseVector<double> poses1(source_id, ids);

  EXPECT_EQ(poses1.source_id(), source_id);
  EXPECT_EQ(poses1.size(), 0);

  const int kCount = 5;
  for (int i = 0; i < kCount; ++i) ids.push_back(FrameId::get_new_id());
  FramePoseVector<double> poses2(source_id, ids);

  EXPECT_EQ(poses2.source_id(), source_id);
  EXPECT_EQ(poses2.size(), kCount);

  FrameId duplicate = FrameId::get_new_id();
  std::vector<FrameId> duplicate_ids{duplicate, FrameId::get_new_id(),
                                     duplicate};
  DRAKE_EXPECT_THROWS_MESSAGE(
      FramePoseVector<double>(source_id, duplicate_ids), std::runtime_error,
      "At least one frame id appears multiple times: \\d+");
}

GTEST_TEST(FrameKinematicsVector, WorkingWithValues) {
  SourceId source_id = SourceId::get_new_id();
  int kPoseCount = 3;
  std::vector<FrameId> ids;
  for (int i = 0; i < kPoseCount; ++i) ids.push_back(FrameId::get_new_id());
  FramePoseVector<double> poses(source_id, ids);

  // Forgot to call clear.
  DRAKE_EXPECT_THROWS_MESSAGE(
      poses.set_value(ids[0], Isometry3<double>::Identity()),
      std::runtime_error,
      "Trying to set kinematics value for the same id .* multiple "
          "times. Did you forget to call clear.*?");

  poses.clear();
  std::vector<Isometry3<double>> recorded_poses;
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    recorded_poses.push_back(pose);
    EXPECT_NO_THROW(poses.set_value(ids[i], pose));
  }

  // Set valid id multiple times.
  DRAKE_EXPECT_THROWS_MESSAGE(
      poses.set_value(ids[0], Isometry3<double>::Identity()),
      std::runtime_error,
      "Trying to set kinematics value for the same id .* multiple "
          "times. Did you forget to call clear.*?");

  // Set invalid id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      poses.set_value(FrameId::get_new_id(), Isometry3<double>::Identity()),
      std::runtime_error,
      "Trying to set a kinematics value for a frame id that does not belong "
          "to the kinematics vector: \\d+");

  // Confirm that poses get recorded properly.
  for (int i = 0; i < kPoseCount; ++i) {
    const Isometry3<double>& pose = poses.value(ids[i]);
    EXPECT_TRUE(CompareMatrices(pose.matrix(), recorded_poses[i].matrix()));
  }

  // Ask for the pose of an id that does not belong to the set.
  DRAKE_EXPECT_THROWS_MESSAGE(poses.value(FrameId::get_new_id()),
                              std::runtime_error,
                              "Can't acquire value for id \\d+. It is not part "
                              "of the kinematics data id set.")
}

GTEST_TEST(FrameKinematicsVector, AutoDiffInstantiation) {
  SourceId source_id = SourceId::get_new_id();
  std::vector<FrameId> ids{FrameId::get_new_id(), FrameId::get_new_id()};
  const int kCount = static_cast<int>(ids.size());
  FramePoseVector<AutoDiffXd> poses(source_id, ids);

  EXPECT_EQ(poses.source_id(), source_id);
  EXPECT_EQ(poses.size(), kCount);
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
