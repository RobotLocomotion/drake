#include "drake/geometry/frame_kinematics_vector.h"

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace test {

using math::RigidTransform;
using math::RigidTransformd;

::testing::AssertionResult ExpectExactIdentity(const RigidTransformd& pose) {
  const RigidTransformd I = RigidTransformd::Identity();
  return CompareMatrices(pose.GetAsMatrix34(), I.GetAsMatrix34());
}

GTEST_TEST(FrameKinematicsVector, DefaultConstructor) {
  const FramePoseVector<double> dut;
  EXPECT_EQ(dut.size(), 0);
}

GTEST_TEST(FrameKinematicsVector, InitializerListCtor) {
  const auto& id_0 = FrameId::get_new_id();
  const RigidTransformd pose_0 = RigidTransformd::Identity();
  const auto& id_1 = FrameId::get_new_id();
  const RigidTransformd pose_1 =
      math::RigidTransformd{Eigen::Translation3d(0.1, 0.2, 0.3)};

  const FramePoseVector<double> dut{{id_0, pose_0}, {id_1, pose_1}};
  ASSERT_EQ(dut.size(), 2);
  EXPECT_TRUE(dut.has_id(id_0));
  EXPECT_TRUE(dut.has_id(id_1));
  EXPECT_FALSE(dut.has_id(FrameId::get_new_id()));
  EXPECT_TRUE(ExpectExactIdentity(dut.value(id_0)));
  EXPECT_TRUE(
      CompareMatrices(dut.value(id_1).GetAsMatrix34(), pose_1.GetAsMatrix34()));
}

GTEST_TEST(FrameKinematicsVector, InitializerListAssign) {
  const auto& id_0 = FrameId::get_new_id();
  const RigidTransformd pose_0 = RigidTransformd::Identity();
  const auto& id_1 = FrameId::get_new_id();
  const RigidTransformd pose_1 =
      math::RigidTransformd{Eigen::Translation3d(0.1, 0.2, 0.3)};

  // Start with a non-empty dut, so we confirm that assignment replaces all of
  // the existing values.  (An STL map insert defaults to a no-op when the key
  // exists already; we don't want that!)
  FramePoseVector<double> dut{{id_1, RigidTransformd::Identity()}};
  dut = {{id_0, pose_0}, {id_1, pose_1}};
  ASSERT_EQ(dut.size(), 2);
  EXPECT_TRUE(dut.has_id(id_0));
  EXPECT_TRUE(dut.has_id(id_1));
  EXPECT_FALSE(dut.has_id(FrameId::get_new_id()));
  EXPECT_TRUE(ExpectExactIdentity(dut.value(id_0)));
  EXPECT_TRUE(
      CompareMatrices(dut.value(id_1).GetAsMatrix34(), pose_1.GetAsMatrix34()));

  dut = {};
  ASSERT_EQ(dut.size(), 0);
}

GTEST_TEST(FrameKinematicsVector, WorkingWithValues) {
  int kPoseCount = 3;
  std::vector<FrameId> ids;
  for (int i = 0; i < kPoseCount; ++i) ids.push_back(FrameId::get_new_id());
  FramePoseVector<double> poses;

  std::vector<RigidTransform<double>> recorded_poses;
  for (int i = 0; i < kPoseCount; ++i) {
    RigidTransformd pose = RigidTransformd::Identity();
    const double d = i;
    pose.set_translation({d, d, d});
    recorded_poses.push_back(pose);
    DRAKE_EXPECT_NO_THROW(poses.set_value(ids[i], pose));
  }

  // Confirm that poses get recorded properly.
  for (int i = 0; i < kPoseCount; ++i) {
    EXPECT_TRUE(poses.has_id(ids[i]));
    const RigidTransformd& pose = poses.value(ids[i]);
    EXPECT_TRUE(CompareMatrices(pose.GetAsMatrix34(),
                                recorded_poses[i].GetAsMatrix34()));
  }

  // Confirm that poses get cleared properly.
  poses.clear();
  EXPECT_EQ(poses.size(), 0);
  EXPECT_FALSE(poses.has_id(ids[0]));

  // Confirm that poses get re-established properly.
  std::reverse(recorded_poses.begin(), recorded_poses.end());
  for (int i = 0; i < kPoseCount; ++i) {
    DRAKE_EXPECT_NO_THROW(poses.set_value(ids[i], recorded_poses[i]));
  }
  for (int i = 0; i < kPoseCount; ++i) {
    EXPECT_TRUE(poses.has_id(ids[i]));
    const RigidTransformd& pose = poses.value(ids[i]);
    EXPECT_TRUE(CompareMatrices(pose.GetAsMatrix34(),
                                recorded_poses[i].GetAsMatrix34()));
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
  const std::vector<RigidTransform<double>> poses{
    RigidTransformd::Identity(),
    RigidTransformd::Identity(),
    RigidTransformd::Identity(),
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
  poses.set_value(FrameId::get_new_id(),
                  RigidTransform<AutoDiffXd>::Identity());
  EXPECT_EQ(poses.size(), 1);
}

GTEST_TEST(FrameKinematicsVector, SymbolicInstantiation) {
  using symbolic::Expression;
  using symbolic::Variable;

  std::vector<FrameId> ids{FrameId::get_new_id(), FrameId::get_new_id()};
  FramePoseVector<Expression> poses;

  // Set and retrieve a simple symbolic::Expression.
  poses.set_value(ids[0], RigidTransform<Expression>::Identity());

  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  const RigidTransform<Expression> pose = RigidTransform<Expression>
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
    poses.set_value(ids.back(), RigidTransformd::Identity());
  }

  std::set<FrameId> actual_ids;
  for (FrameId id : poses.frame_ids()) actual_ids.insert(id);

  EXPECT_EQ(ids.size(), actual_ids.size());
  for (FrameId id : ids) EXPECT_EQ(actual_ids.count(id), 1);
}

}  // namespace test
}  // namespace geometry
}  // namespace drake
