#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace {

GTEST_TEST(testIKpointwise, simpleIKpointwise) {
  RigidBodyTree model(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf");
  Vector2d tspan;
  tspan << 0, 1;
  int nT = 3;
  double t[3] = {0.0, 0.5, 1.0};
  MatrixXd q0(model.get_num_positions(), nT);
  for (int i = 0; i < nT; i++) {
    q0.col(i) = model.getZeroConfiguration();
    q0(3, i) = 0.8;
  }
  Vector3d com_des = Vector3d::Zero();
  com_des(2) = std::numeric_limits<double>::quiet_NaN();
  WorldCoMConstraint com_kc(&model, com_des, com_des);
  Vector3d com_lb = Vector3d::Zero();
  Vector3d com_ub = Vector3d::Zero();
  com_lb(2) = 0.9;
  com_ub(2) = 1.0;
  Vector2d tspan_end;
  tspan_end << 0.9, 1;
  WorldCoMConstraint com_kc_final(&model, com_lb, com_ub, tspan_end);

  int num_constraints = 2;
  RigidBodyConstraint** constraint_array =
      new RigidBodyConstraint* [num_constraints];
  constraint_array[0] = &com_kc;
  constraint_array[1] = &com_kc_final;
  IKoptions ikoptions(&model);
  MatrixXd q_sol(model.get_num_positions(), nT);
  int* info = new int[nT];
  std::vector<std::string> infeasible_constraint;
  inverseKinPointwise(&model, nT, t, q0, q0, num_constraints, constraint_array,
                      ikoptions, &q_sol, info, &infeasible_constraint);
  for (int i = 0; i < nT; i++) {
    printf("info[%d] = %d ", i, info[i]);
    EXPECT_EQ(info[i], 1);
  }
  printf("\n");

  const Vector3d expected_initial(0, 0, 0.216933);
  const Vector3d expected_final(0, 0, 0.9);
  for (int i = 0; i < nT; i++) {
    KinematicsCache<double> cache = model.doKinematics(q_sol.col(i));
    Vector3d com = model.centerOfMass(cache);
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
