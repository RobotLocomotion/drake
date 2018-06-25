/* clang-format off to disable clang-format-includes */
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/joints/fixed_axis_one_dof_joint.h"
/* clang-format on */

#include <cmath>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

using std::unique_ptr;
using Eigen::VectorXd;
namespace drake {

using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace plants {
namespace test {
namespace {

class RigidBodyTreeSpringTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_ = std::make_unique<RigidBodyTree<double>>();
  }
  unique_ptr<RigidBodyTree<double>> tree_;
};

// Tests spring forces effect on RigidBodyTree dynamics
// Computes dynamics with and without a spring, and checks that the difference
// is correct
TEST_F(RigidBodyTreeSpringTest, QuaternionBaseSpringTest) {
  const std::string filename = FindResourceOrThrow(
      "drake/multibody/test/rigid_body_tree/two_dof_robot.urdf");
  AddModelInstanceFromUrdfFileToWorld(filename, multibody::joints::kQuaternion,
    tree_.get());


  //compute bias without spring forces
  VectorXd q = VectorXd::Random(tree_->get_num_positions());
  VectorXd v = VectorXd::Random(tree_->get_num_velocities());


  auto kinsol = tree_->doKinematics(q, v);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd bias_no_spring = tree_->dynamicsBiasTerm(kinsol,
                                                    no_external_wrenches, true);

  double stiffness = 102.0;
  double nominal_position = 1.1;

  int body_index = tree_->FindIndexOfChildBodyOfJoint("joint2");
  auto body = tree_->get_mutable_body(body_index);

  FixedAxisOneDoFJoint<double>* joint =
      static_cast<FixedAxisOneDoFJoint<double>*>(body->get_mutable_joint());

  EXPECT_TRUE(joint != nullptr);

  joint->SetSpringDynamics(stiffness, nominal_position);

  VectorXd bias_spring = tree_->dynamicsBiasTerm(kinsol,
                                                 no_external_wrenches, true);
  VectorXd delta_bias = VectorXd::Zero(tree_->get_num_velocities());
  int q_ind = body->get_position_start_index();
  int v_ind = body->get_velocity_start_index();
  delta_bias(v_ind) = stiffness * (nominal_position - q(q_ind));

  EXPECT_TRUE(CompareMatrices(bias_no_spring - delta_bias, bias_spring,
    1e-20, MatrixCompareType::absolute));
}

TEST_F(RigidBodyTreeSpringTest, FixedBaseSpringTest) {
  const std::string filename = FindResourceOrThrow(
      "drake/multibody/test/rigid_body_tree/two_dof_robot.urdf");
  AddModelInstanceFromUrdfFileToWorld(filename, multibody::joints::kFixed,
    tree_.get());


  //compute bias without spring forces
  VectorXd q = VectorXd::Random(tree_->get_num_positions());
  VectorXd v = VectorXd::Random(tree_->get_num_velocities());


  auto kinsol = tree_->doKinematics(q, v);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd bias_no_spring = tree_->dynamicsBiasTerm(kinsol,
                                                    no_external_wrenches, true);

  double stiffness = 102.0;
  double nominal_position = 1.1;

  int body_index = tree_->FindIndexOfChildBodyOfJoint("joint2");
  auto body = tree_->get_mutable_body(body_index);

  FixedAxisOneDoFJoint<double>* joint =
      static_cast<FixedAxisOneDoFJoint<double>*>(body->get_mutable_joint());

  EXPECT_TRUE(joint != nullptr);

  joint->SetSpringDynamics(stiffness, nominal_position);

  VectorXd bias_spring = tree_->dynamicsBiasTerm(kinsol,
                                                 no_external_wrenches, true);
  VectorXd delta_bias = VectorXd::Zero(tree_->get_num_velocities());
  int q_ind = body->get_position_start_index();
  int v_ind = body->get_velocity_start_index();
  delta_bias(v_ind) = stiffness * (nominal_position - q(q_ind));

  EXPECT_TRUE(CompareMatrices(bias_no_spring - delta_bias, bias_spring,
    1e-20, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
