#include "drake/geometry/frame_kinematics_vector.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace test {

::testing::AssertionResult ExpectExactIdentity(const Isometry3<double>& pose) {
  const Isometry3<double> I = Isometry3<double>::Identity();
  return CompareMatrices(pose.matrix().block<3, 4>(0, 0),
                         I.matrix().block<3, 4>(0, 0));
}

GTEST_TEST(FrameKinematicsVector, DefaultConstructor) {
  const FramePoseVector<double> dut;
  EXPECT_FALSE(dut.source_id().is_valid());
  EXPECT_EQ(dut.size(), 0);
}

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

  // Confirm that the values are properly initialized.
  for (int i = 0; i < kCount; ++i) {
    EXPECT_TRUE(ExpectExactIdentity(poses2.value(ids[i])));
  }

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

GTEST_TEST(FrameKinematicsVector, SymbolicInstantiation) {
  using symbolic::Expression;
  using symbolic::Variable;

  SourceId source_id = SourceId::get_new_id();
  std::vector<FrameId> ids{FrameId::get_new_id(), FrameId::get_new_id()};
  const int kCount = static_cast<int>(ids.size());
  FramePoseVector<Expression> poses(source_id, ids);

  EXPECT_EQ(poses.source_id(), source_id);
  EXPECT_EQ(poses.size(), kCount);

  // Set and retrieve a simple symbolic::Expression.
  poses.clear();

  poses.set_value(ids[0], Isometry3<Expression>::Identity());

  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  const Isometry3<Expression> pose = Isometry3<Expression>
      (Translation3<Expression>(x_, y_, z_));
  poses.set_value(ids[1], pose);

  EXPECT_TRUE(x_.EqualTo(poses.value(ids[1]).translation()[0]));
  EXPECT_TRUE(y_.EqualTo(poses.value(ids[1]).translation()[1]));
  EXPECT_TRUE(z_.EqualTo(poses.value(ids[1]).translation()[2]));
}

GTEST_TEST(FrameKinematicsVector, FrameIdRange) {
  SourceId source_id = SourceId::get_new_id();
  int kPoseCount = 3;
  std::vector<FrameId> ids;
  for (int i = 0; i < kPoseCount; ++i) ids.push_back(FrameId::get_new_id());
  FramePoseVector<double> poses(source_id, ids);

  std::set<FrameId> actual_ids;
  for (FrameId id : poses.frame_ids()) actual_ids.insert(id);

  EXPECT_EQ(ids.size(), actual_ids.size());
  for (FrameId id : ids) EXPECT_EQ(actual_ids.count(id), 1);
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
