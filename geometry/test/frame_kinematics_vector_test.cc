#include "drake/geometry/frame_kinematics_vector.h"

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/rigid_transform.h"

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_FALSE(dut.source_id().is_valid());
#pragma GCC diagnostic pop
  EXPECT_EQ(dut.size(), 0);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
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
#pragma GCC diagnostic pop

GTEST_TEST(FrameKinematicsVector, InitializerListCtor) {
  const auto& id_0 = FrameId::get_new_id();
  const Isometry3<double> pose_0 = Isometry3<double>::Identity();
  const auto& id_1 = FrameId::get_new_id();
  const Isometry3<double> pose_1 =
      math::RigidTransformd{Eigen::Translation3d(0.1, 0.2, 0.3)}.
          GetAsIsometry3();

  const FramePoseVector<double> dut{{id_0, pose_0}, {id_1, pose_1}};
  ASSERT_EQ(dut.size(), 2);
  EXPECT_TRUE(dut.has_id(id_0));
  EXPECT_TRUE(dut.has_id(id_1));
  EXPECT_FALSE(dut.has_id(FrameId::get_new_id()));
  EXPECT_TRUE(ExpectExactIdentity(dut.value(id_0)));
  EXPECT_TRUE(CompareMatrices(
      dut.value(id_1).matrix().block<3, 4>(0, 0),
      pose_1.matrix().block<3, 4>(0, 0)));
}

GTEST_TEST(FrameKinematicsVector, InitializerListAssign) {
  const auto& id_0 = FrameId::get_new_id();
  const Isometry3<double> pose_0 = Isometry3<double>::Identity();
  const auto& id_1 = FrameId::get_new_id();
  const Isometry3<double> pose_1 =
      math::RigidTransformd{Eigen::Translation3d(0.1, 0.2, 0.3)}.
          GetAsIsometry3();

  // Start with a non-empty dut, so we confirm that assignment replaces all of
  // the existing values.  (An STL map insert defaults to a no-op when the key
  // exists already; we don't want that!)
  FramePoseVector<double> dut{{id_1, Isometry3<double>::Identity()}};
  dut = {{id_0, pose_0}, {id_1, pose_1}};
  ASSERT_EQ(dut.size(), 2);
  EXPECT_TRUE(dut.has_id(id_0));
  EXPECT_TRUE(dut.has_id(id_1));
  EXPECT_FALSE(dut.has_id(FrameId::get_new_id()));
  EXPECT_TRUE(ExpectExactIdentity(dut.value(id_0)));
  EXPECT_TRUE(CompareMatrices(
      dut.value(id_1).matrix().block<3, 4>(0, 0),
      pose_1.matrix().block<3, 4>(0, 0)));

  dut = {};
  ASSERT_EQ(dut.size(), 0);
}

GTEST_TEST(FrameKinematicsVector, WorkingWithValues) {
  int kPoseCount = 3;
  std::vector<FrameId> ids;
  for (int i = 0; i < kPoseCount; ++i) ids.push_back(FrameId::get_new_id());
  FramePoseVector<double> poses;

  std::vector<Isometry3<double>> recorded_poses;
  for (int i = 0; i < kPoseCount; ++i) {
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << i, i, i;
    recorded_poses.push_back(pose);
    EXPECT_NO_THROW(poses.set_value(ids[i], pose));
  }

  // Confirm that poses get recorded properly.
  for (int i = 0; i < kPoseCount; ++i) {
    EXPECT_TRUE(poses.has_id(ids[i]));
    const Isometry3<double>& pose = poses.value(ids[i]);
    EXPECT_TRUE(CompareMatrices(pose.matrix(), recorded_poses[i].matrix()));
  }

  // Confirm that poses get cleared properly.
  poses.clear();
  EXPECT_EQ(poses.size(), 0);
  EXPECT_FALSE(poses.has_id(ids[0]));

  // Confirm that poses get re-established properly.
  std::reverse(recorded_poses.begin(), recorded_poses.end());
  for (int i = 0; i < kPoseCount; ++i) {
    EXPECT_NO_THROW(poses.set_value(ids[i], recorded_poses[i]));
  }
  for (int i = 0; i < kPoseCount; ++i) {
    EXPECT_TRUE(poses.has_id(ids[i]));
    const Isometry3<double>& pose = poses.value(ids[i]);
    EXPECT_TRUE(CompareMatrices(pose.matrix(), recorded_poses[i].matrix()));
  }

  // Ask for the pose of an id that does not belong to the set.
  DRAKE_EXPECT_THROWS_MESSAGE(poses.value(FrameId::get_new_id()),
                              std::runtime_error,
                              "No such FrameId \\d+.");
}

GTEST_TEST(FrameKinematicsVector, SetWithoutAllocations) {
  const int kPoseCount = 3;
  const std::vector<FrameId> ids{
    FrameId::get_new_id(),
    FrameId::get_new_id(),
    FrameId::get_new_id(),
  };
  const std::vector<Isometry3<double>> poses{
    Isometry3<double>::Identity(),
    Isometry3<double>::Identity(),
    Isometry3<double>::Identity(),
  };

  // For the initial setting, we'd expect to see allocations.
  FramePoseVector<double> dut;
  for (int i = 0; i < kPoseCount; ++i) {
    dut.set_value(ids[i], poses[i]);
  }

  // Subsequent clear + set should not touch the heap.
  for (int j = 0; j < 5; ++j) {
    drake::test::LimitMalloc guard;
    dut.clear();
    for (int i = 0; i < kPoseCount; ++i) {
      dut.set_value(ids[i], poses[i]);
    }
  }
}

GTEST_TEST(FrameKinematicsVector, AutoDiffInstantiation) {
  FramePoseVector<AutoDiffXd> poses;
  poses.set_value(FrameId::get_new_id(), Isometry3<AutoDiffXd>::Identity());
  EXPECT_EQ(poses.size(), 1);
}

GTEST_TEST(FrameKinematicsVector, SymbolicInstantiation) {
  using symbolic::Expression;
  using symbolic::Variable;

  std::vector<FrameId> ids{FrameId::get_new_id(), FrameId::get_new_id()};
  FramePoseVector<Expression> poses;

  // Set and retrieve a simple symbolic::Expression.
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
  FramePoseVector<double> poses;
  std::vector<FrameId> ids;
  for (int i = 0; i < 3; ++i) {
    ids.push_back(FrameId::get_new_id());
    poses.set_value(ids.back(), Isometry3<double>::Identity());
  }

  std::set<FrameId> actual_ids;
  for (FrameId id : poses.frame_ids()) actual_ids.insert(id);

  EXPECT_EQ(ids.size(), actual_ids.size());
  for (FrameId id : ids) EXPECT_EQ(actual_ids.count(id), 1);
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
