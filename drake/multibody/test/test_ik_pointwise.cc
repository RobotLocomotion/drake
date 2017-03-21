#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace {

GTEST_TEST(testIKpointwise, simpleIKpointwise) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf",
      multibody::joints::kRollPitchYaw, tree.get());

  Vector2d tspan;
  tspan << 0, 1;
  int nT = 3;
  double t[3] = {0.0, 0.5, 1.0};
  MatrixXd q0(tree->get_num_positions(), nT);
  for (int i = 0; i < nT; i++) {
    q0.col(i) = tree->getZeroConfiguration();
    q0(3, i) = 0.8;
  }
  Vector3d com_des(0, 0, std::numeric_limits<double>::quiet_NaN());
  WorldCoMConstraint com_kc(tree.get(), com_des, com_des);
  Vector3d com_lb(0, 0, 0.9);
  Vector3d com_ub(0, 0, 1.0);
  Vector2d tspan_end(0.9, 1);
WorldCoMConstraint com_kc_final(tree.get(), com_lb, com_ub, tspan_end);

  int num_constraints = 2;
  RigidBodyConstraint** constraint_array =
      new RigidBodyConstraint* [num_constraints];
  constraint_array[0] = &com_kc;
  constraint_array[1] = &com_kc_final;
  IKoptions ikoptions(tree.get());
  MatrixXd q_sol(tree->get_num_positions(), nT);
  int* info = new int[nT];
  std::vector<std::string> infeasible_constraint;
  inverseKinPointwise(tree.get(), nT, t, q0, q0, num_constraints,
                      constraint_array, ikoptions, &q_sol, info,
                      &infeasible_constraint);
  for (int i = 0; i < nT; i++) {
    printf("info[%d] = %d ", i, info[i]);
    EXPECT_EQ(info[i], 1);
  }
  printf("\n");

  const Vector3d expected_initial(0, 0, 0.216933);
  const Vector3d expected_final(0, 0, 0.9);
  for (int i = 0; i < nT; i++) {
    KinematicsCache<double> cache = tree->doKinematics(q_sol.col(i));
    Vector3d com = tree->centerOfMass(cache);
    printf("t %d: %5.6f\n%5.6f\n%5.6f\n", i, com(0), com(1), com(2));
    if (i < (nT - 1)) {
      // SNOPT and IPOPT diverge slightly in their output, so reduce
      // the tolerance a bit.
      EXPECT_TRUE(CompareMatrices(com, expected_initial, 1e-4,
                                  MatrixCompareType::absolute));
    } else {
      EXPECT_TRUE(CompareMatrices(com, expected_final, 1e-6,
                                  MatrixCompareType::absolute));
    }
  }

  delete[] constraint_array;
  delete[] info;
}

}  // namespace
}  // namespace drake
