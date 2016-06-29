#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>

#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/util/eigen_matrix_compare.h"
#include "gtest/gtest.h"

using namespace std;
using namespace Eigen;
using drake::util::CompareMatrices;
using drake::util::MatrixCompareType;

GTEST_TEST(testIK, atlasIK) {
  RigidBodyTree model("examples/Atlas/urdf/atlas_minimal_contact.urdf");

  Vector2d tspan;
  tspan << 0, 1;
  VectorXd q0 = model.getZeroConfiguration();
  // The state frame of cpp model does not match with the state frame of MATLAB
  // model, since the dofname_to_dofnum is different in cpp and MATLAB
  q0(2) = 0.8;
  Vector3d com_lb = Vector3d::Zero();
  Vector3d com_ub = Vector3d::Zero();
  com_lb(2) = 0.9;
  com_ub(2) = 1.0;
  WorldCoMConstraint com_kc(&model, com_lb, com_ub, tspan);

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&com_kc);
  IKoptions ikoptions(&model);
  VectorXd q_sol(model.number_of_positions());
  int info;
  vector<string> infeasible_constraint;
  inverseKin(&model, q0, q0, constraint_array.size(), constraint_array.data(),
             ikoptions, &q_sol, &info, &infeasible_constraint);
  printf("INFO = %d\n", info);
  EXPECT_EQ(info, 1);

  KinematicsCache<double> cache = model.doKinematics(q_sol);
  Vector3d com = model.centerOfMass(cache);
  printf("%5.6f\n%5.6f\n%5.6f\n", com(0), com(1), com(2));
  EXPECT_TRUE(
      CompareMatrices(com, Vector3d(0, 0, 1), 1e-6,
                      MatrixCompareType::absolute));

  /*MATFile *presultmat;
  presultmat = matOpen("q_sol.mat","w");
  mxArray* pqsol = mxCreateDoubleMatrix(model.num_dof, 1, mxREAL);
  memcpy(mxGetPrSafe(pqsol), q_sol.data(), sizeof(double)model.num_dof);
  matPutVariable(presultmat,"q_sol", pqsol);
  matClose(presultmat);*/
}

GTEST_TEST(testIK, iiwaIK) {
  RigidBodyTree model("examples/kuka_iiwa_arm/urdf/iiwa14.urdf");

  Vector2d tspan;
  tspan << 0, 1;
  VectorXd q0 = model.getZeroConfiguration();

  Vector3d pos_end;
  pos_end << -0.4, 0, 1.1;
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.1);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.1);
  WorldPositionConstraint wpc(&model, model.FindBodyIndex("iiwa_link_7"),
                              pos_end, pos_lb, pos_ub, tspan);

  PostureConstraint pc(&model, tspan);
  Vector3d joint_lb = Vector3d::Zero();
  joint_lb(0) = 0.9;
  joint_lb(1) = 0.9;
  joint_lb(2) = 0.9;

  Vector3d joint_ub = Vector3d::Zero();
  joint_ub(0) = 1.0;
  joint_ub(1) = 1.0;
  joint_ub(2) = 1.0;

  Eigen::VectorXi joint_idx(1);
  joint_idx(0) = model.findJointId("iiwa_joint_2");
  pc.setJointLimits(joint_idx, joint_lb, joint_ub);

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&wpc);
  constraint_array.push_back(&pc);
  IKoptions ikoptions(&model);
  VectorXd q_sol(model.number_of_positions());
  int info;
  vector<string> infeasible_constraint;
  inverseKin(&model, q0, q0, constraint_array.size(), constraint_array.data(),
             ikoptions, &q_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);

  VectorXd expected(7);
  expected << 0.096184, 1.4963, 0.0144598, 0.9,
      0.0296464, -1.44644, 0.00662504;
  EXPECT_TRUE(
      CompareMatrices(q_sol, expected, 1e-5, MatrixCompareType::absolute));
}
