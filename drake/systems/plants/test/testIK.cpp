#include <cstdlib>
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

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace {

GTEST_TEST(testIK, atlasIK) {
  RigidBodyTree model(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf");

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
  VectorXd q_sol(model.get_num_positions());
  q_sol.setZero();
  int info = 0;
  std::vector<std::string> infeasible_constraint;
  inverseKin(&model, q0, q0, constraint_array.size(), constraint_array.data(),
             ikoptions, &q_sol, &info, &infeasible_constraint);
  printf("info = %d\n", info);
  EXPECT_EQ(info, 1);

  KinematicsCache<double> cache = model.doKinematics(q_sol);
  Vector3d com = model.centerOfMass(cache);
  printf("%5.6f\n%5.6f\n%5.6f\n", com(0), com(1), com(2));
  EXPECT_TRUE(
      CompareMatrices(com, Vector3d(0, 0, 1), 1e-6,
                      MatrixCompareType::absolute));
}

GTEST_TEST(testIK, iiwaIK) {
  RigidBodyTree model(
      GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf");

  // Create a timespan for the constraints.  It's not particularly
  // meaningful in this test since inverseKin() only tests a single
  // point, but the constructors need something.
  Vector2d tspan;
  tspan << 0, 1;

  // Start the robot in the zero configuration (all joints zeroed,
  // pointing straight up).
  VectorXd q0 = model.getZeroConfiguration();

  // Constrain iiwa_link_7 (the end effector) to move 0.58 on the X
  // axis and down slightly (to make room for the X axis motion).
  Vector3d pos_end;
  pos_end << 0.58, 0, 0.77;
  const double pos_tol = 0.01;
  Vector3d pos_lb = pos_end - Vector3d::Constant(pos_tol);
  Vector3d pos_ub = pos_end + Vector3d::Constant(pos_tol);
  const int link_7_idx = model.FindBodyIndex("iiwa_link_7");
  WorldPositionConstraint wpc(&model, link_7_idx,
                              Vector3d(0, 0, 0), pos_lb, pos_ub, tspan);

  // Constrain iiwa_joint_4 between 0.9 and 1.0.
  PostureConstraint pc(&model, tspan);
  drake::Vector1d joint_lb(0.9);
  drake::Vector1d joint_ub(1.0);

  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) = model.FindChildBodyOfJoint("iiwa_joint_4")->
      get_position_start_index();
  pc.setJointLimits(joint_position_start_idx, joint_lb, joint_ub);

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&wpc);
  constraint_array.push_back(&pc);
  IKoptions ikoptions(&model);

  VectorXd q_sol(model.get_num_positions());
  q_sol.setZero();
  int info = 0;
  std::vector<std::string> infeasible_constraint;
  inverseKin(&model, q0, q0, constraint_array.size(), constraint_array.data(),
             ikoptions, &q_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);

  // Check that our constrained joint is within where we tried to constrain it.
  EXPECT_GE(q_sol(joint_position_start_idx(0)), joint_lb(0));
  EXPECT_LE(q_sol(joint_position_start_idx(0)), joint_ub(0));

  // Check that the link we were trying to position wound up where we expected.
  KinematicsCache<double> cache = model.doKinematics(q_sol);
  EXPECT_TRUE(CompareMatrices(
      pos_end, model.relativeTransform(cache, 0, link_7_idx).translation(),
      pos_tol + 1e-6, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace drake
