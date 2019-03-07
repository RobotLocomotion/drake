/* clang-format off to disable clang-format-includes */
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/joints/revolute_joint.h"
/* clang-format on */

#include <cmath>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace systems {
namespace plants {
namespace test {
namespace {

using std::unique_ptr;
using Eigen::VectorXd;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

class RigidBodyTreeSpringTest : public ::testing::Test {
 protected:
  void TestTreeWithSpring(bool use_fixed_base) {
    unique_ptr<RigidBodyTree<double>> tree =
        std::make_unique<RigidBodyTree<double>>();
    const std::string filename = FindResourceOrThrow(
        "drake/multibody/test/rigid_body_tree/two_dof_robot.urdf");
    if (use_fixed_base) {
      AddModelInstanceFromUrdfFileToWorld(filename,
          multibody::joints::kQuaternion, tree.get());
    } else {
      AddModelInstanceFromUrdfFileToWorld(filename, multibody::joints::kFixed,
          tree.get());
    }

    // Compute bias without spring forces.
    VectorXd q(tree->get_num_positions());
    VectorXd v(tree->get_num_velocities());
    for (int i = 0; i < q.size(); ++i) {
      q(i) = i+1;
    }
    for (int i = 0; i < v.size(); ++i) {
      v(i) = -2*i-1;
    }


    KinematicsCache<double> kinsol = tree->doKinematics(q, v);
    const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
    VectorXd bias_no_spring = tree->dynamicsBiasTerm(kinsol,
                                                     no_external_wrenches,
                                                     true);

    const double stiffness = 102.0;  // Nm/rad
    const double nominal_position = 1.1;  // rad

    const int body_index = tree->FindIndexOfChildBodyOfJoint("joint2");
    auto body = tree->get_mutable_body(body_index);

    RevoluteJoint& joint = dynamic_cast<RevoluteJoint&>(
        body->get_mutable_joint());

    joint.SetSpringDynamics(stiffness, nominal_position);

    VectorXd bias_spring = tree->dynamicsBiasTerm(kinsol,
                                                  no_external_wrenches, true);
    VectorXd delta_bias = VectorXd::Zero(tree->get_num_velocities());
    const int q_ind = body->get_position_start_index();
    const int v_ind = body->get_velocity_start_index();
    delta_bias(v_ind) = stiffness * (nominal_position - q(q_ind));

    EXPECT_TRUE(CompareMatrices(bias_no_spring - delta_bias, bias_spring,
      0, MatrixCompareType::absolute));
  }
};

// Tests spring forces effect on RigidBodyTree dynamics.
// Computes dynamics with and without a spring, and checks that the difference
// is correct.
TEST_F(RigidBodyTreeSpringTest, QuaternionBaseSpringTest) {
  TestTreeWithSpring(false);
}

TEST_F(RigidBodyTreeSpringTest, FixedBaseSpringTest) {
  TestTreeWithSpring(true);
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake
