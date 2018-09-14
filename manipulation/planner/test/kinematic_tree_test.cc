#include "drake/manipulation/planner/kinematic_tree.h"

#include <random>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

using testing::Return;
using testing::_;

namespace drake {
namespace manipulation {
namespace planner {
namespace {
class MockKinematicTree : public KinematicTree {
 public:
  MOCK_CONST_METHOD0(num_positions, int());
  MOCK_CONST_METHOD0(num_velocities, int());
  MOCK_CONST_METHOD1(PositionStartIndexForBody, int(const std::string&));
  MOCK_CONST_METHOD0(joint_position_lower_limit,
                     const drake::VectorX<double>&());
  MOCK_CONST_METHOD0(joint_position_upper_limit,
                     const drake::VectorX<double>&());
  MOCK_CONST_METHOD0(joint_velocity_lower_limit,
                     const drake::VectorX<double>&());
  MOCK_CONST_METHOD0(joint_velocity_upper_limit,
                     const drake::VectorX<double>&());
  MOCK_CONST_METHOD0(GetZeroConfiguration, drake::VectorX<double>());
  MOCK_CONST_METHOD1(GetRandomConfiguration,
                     drake::VectorX<double>(std::default_random_engine*));
  MOCK_CONST_METHOD3(
      CalcRelativeTransform,
      drake::math::Transform<double>(const drake::VectorX<double>&,
                                     const std::string&, const std::string&));
  MOCK_CONST_METHOD1(MakeCollisionAvoidanceConstraint,
                     std::shared_ptr<drake::solvers::Constraint>(double));
  MOCK_CONST_METHOD5(MakeRelativePoseConstraint,
                     std::shared_ptr<drake::solvers::Constraint>(
                         const std::string&, const std::string&,
                         const drake::math::Transform<double>&, double,
                         double));
  MOCK_METHOD3(DoSetJointPositionLimits,
               void(int position_index, double lower_limit,
                    double upper_limit));
  MOCK_METHOD3(DoSetJointVelocityLimits,
               void(int velocity_index, double lower_limit,
                    double upper_limit));
};

// Verify that SetJointPositionLimits() calls DoSetJointPositionLimits() with
// the proper arguments.
GTEST_TEST(KinematicTreeTests, SetJointPositionLimitsScalarTest) {
  MockKinematicTree tree;
  const int num_positions{5};
  EXPECT_CALL(tree, num_positions()).WillRepeatedly(Return(num_positions));
  const int expected_position_index{3};
  const double expected_lower_limit{-1};
  const double expected_upper_limit{1};
  EXPECT_CALL(tree, DoSetJointPositionLimits(expected_position_index,
                                             expected_lower_limit,
                                             expected_upper_limit));
  tree.SetJointPositionLimits(expected_position_index, expected_lower_limit,
                              expected_upper_limit);
}

// Verify that the vector version of SetJointPositionLimits() calls
// DoSetJointPositionLimits() the appropriate number of times.
GTEST_TEST(KinematicTreeTests, SetJointPositionLimitsVectorTest) {
  MockKinematicTree tree;
  const int num_positions{5};
  auto lower_limit = -VectorX<double>::LinSpaced(num_positions, 0, 1);
  auto upper_limit = VectorX<double>::LinSpaced(num_positions, 0, 1);
  EXPECT_CALL(tree, num_positions()).WillRepeatedly(Return(num_positions));
  EXPECT_CALL(tree, DoSetJointPositionLimits(_, _, _)).Times(num_positions);
  tree.SetJointPositionLimits(lower_limit, upper_limit);
}

// Verify that SetJointVelocityLimits() calls DoSetJointVelocityLimits() with
// the proper arguments.
GTEST_TEST(KinematicTreeTests, SetJointVelocityLimitsScalarTest) {
  MockKinematicTree tree;
  const int num_velocities{5};
  EXPECT_CALL(tree, num_velocities()).WillRepeatedly(Return(num_velocities));
  const int expected_velocity_index{3};
  const double expected_lower_limit{-1};
  const double expected_upper_limit{1};
  EXPECT_CALL(tree, DoSetJointVelocityLimits(expected_velocity_index,
                                             expected_lower_limit,
                                             expected_upper_limit));
  tree.SetJointVelocityLimits(expected_velocity_index, expected_lower_limit,
                              expected_upper_limit);
}

// Verify that the vector version of SetJointVelocityLimits() calls
// DoSetJointVelocityLimits() the appropriate number of times.
GTEST_TEST(KinematicTreeTests, SetJointVelocityLimitsVectorTest) {
  MockKinematicTree tree;
  const int num_velocities{5};
  auto lower_limit = -VectorX<double>::LinSpaced(num_velocities, 0, 1);
  auto upper_limit = VectorX<double>::LinSpaced(num_velocities, 0, 1);
  EXPECT_CALL(tree, num_velocities()).WillRepeatedly(Return(num_velocities));
  EXPECT_CALL(tree, DoSetJointVelocityLimits(_, _, _)).Times(num_velocities);
  tree.SetJointVelocityLimits(lower_limit, upper_limit);
}
}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
