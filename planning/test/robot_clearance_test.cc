#include "drake/planning/robot_clearance.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace planning {
namespace {

using multibody::BodyIndex;
using testing::ElementsAre;

GTEST_TEST(RobotClearanceTest, Empty) {
  const int nq = 7;
  RobotClearance dut(nq);
  EXPECT_EQ(dut.size(), 0);
  EXPECT_EQ(dut.num_positions(), nq);
  EXPECT_EQ(dut.robot_indices().size(), 0);
  EXPECT_EQ(dut.other_indices().size(), 0);
  EXPECT_EQ(dut.collision_types().size(), 0);
  EXPECT_EQ(dut.distances().size(), 0);
  EXPECT_EQ(dut.jacobians().size(), 0);
  EXPECT_EQ(dut.jacobians().cols(), nq);
}

GTEST_TEST(RobotClearanceTest, AppendData) {
  const int nq = 7;
  RobotClearance dut(nq);

  // Reserve with size==2, but we'll only use size==1.
  dut.Reserve(2);

  const BodyIndex robot_body{1};
  const BodyIndex env_body{2};
  const RobotCollisionType type{RobotCollisionType::kEnvironmentCollision};
  const double phi{2.0};
  const Eigen::RowVectorXd dq{Eigen::RowVectorXd::LinSpaced(nq, 0.0, 1.0)};

  dut.Append(robot_body, env_body, type, phi, dq);
  EXPECT_EQ(dut.size(), 1);
  EXPECT_EQ(dut.num_positions(), nq);
  EXPECT_THAT(dut.robot_indices(), ElementsAre(robot_body));
  EXPECT_THAT(dut.other_indices(), ElementsAre(env_body));
  EXPECT_THAT(dut.collision_types(), ElementsAre(type));
  ASSERT_EQ(dut.distances().size(), 1);
  EXPECT_EQ(dut.distances()[0], phi);
  ASSERT_EQ(dut.jacobians().rows(), 1);
  EXPECT_TRUE(CompareMatrices(dut.jacobians().row(0), dq));

  // Zero out one column (to check the mutable accessor function).
  EXPECT_NE(dut.jacobians()(0, 3), 0.0);
  dut.mutable_jacobians().col(3).setZero();
  EXPECT_EQ(dut.jacobians()(0, 3), 0.0);
}

GTEST_TEST(RobotClearanceTest, AppendThrow) {
  const int nq = 7;
  RobotClearance dut(nq);

  const BodyIndex body{1};
  const RobotCollisionType type{RobotCollisionType::kEnvironmentCollision};
  const Eigen::RowVectorXd dq{Eigen::RowVectorXd::LinSpaced(nq, 0, 1)};

  // Wrong sized jacobian throws.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.Append(body, body, type, 0.0, dq.col(0)),
                              ".*jacobian.*nq.*");

  // Right sized jacobian does not throw.
  EXPECT_NO_THROW(dut.Append(body, body, type, 0.0, dq));
}

}  // namespace
}  // namespace planning
}  // namespace drake
