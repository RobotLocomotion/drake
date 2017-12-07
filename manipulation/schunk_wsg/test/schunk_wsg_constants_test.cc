#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

// Test that the constants defined in schunk_wsg_constants.h are
// correct wrt `models/schunk_wsg_50.sdf`.
GTEST_TEST(SchunkWsgConstantTest, ConstantTest) {
  RigidBodyTree<double> wsg;
  parsers::sdf::AddModelInstancesFromSdfFile(
      FindResourceOrThrow(
          "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf"),
      multibody::joints::kFixed, nullptr, &wsg);

  const std::map<std::string, int> index_map =
      wsg.computePositionNameToIndexMap();

  const int position_index = index_map.at("left_finger_sliding_joint");

  const int velocity_index = position_index + wsg.get_num_positions();

  EXPECT_EQ(wsg.get_num_positions(), kSchunkWsgNumPositions);
  EXPECT_EQ(wsg.get_num_velocities(), kSchunkWsgNumVelocities);
  EXPECT_EQ(wsg.get_num_actuators(), kSchunkWsgNumActuators);

  EXPECT_EQ(position_index, kSchunkWsgPositionIndex);
  EXPECT_EQ(velocity_index, kSchunkWsgVelocityIndex);

  MatrixX<double> feedback_matrix = MatrixX<double>::Zero(
      2 * wsg.get_num_actuators(), 2 * wsg.get_num_positions());
  feedback_matrix(0, position_index) = 1.;
  feedback_matrix(1, velocity_index) = 1.;

  EXPECT_TRUE(drake::CompareMatrices(
      feedback_matrix, GetSchunkWsgFeedbackSelector<double>(), 1e-15,
      drake::MatrixCompareType::absolute));
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
