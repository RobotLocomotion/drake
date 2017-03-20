#include <cstdlib>
#include <memory>
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

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace {

GTEST_TEST(testIK, atlasIK) {
  auto model = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf",
      drake::multibody::joints::kRollPitchYaw, model.get());

  const Vector2d tspan(0, 1);
  VectorXd q0 = model->getZeroConfiguration();
  // The state frame of cpp model does not match with the state frame of MATLAB
  // model, since the dofname_to_dofnum is different in cpp and MATLAB
  q0(2) = 0.8;
  const Vector3d com_lb(0, 0, 0.9);
  const Vector3d com_ub(0, 0, 1.0);
  const WorldCoMConstraint com_kc(model.get(), com_lb, com_ub, tspan);

  const std::vector<const RigidBodyConstraint*> constraint_array{&com_kc};
  const IKoptions ikoptions(model.get());
  VectorXd q_sol = VectorXd::Zero(model->get_num_positions());
  int info = 0;
  std::vector<std::string> infeasible_constraint;
  inverseKin(model.get(), q0, q0, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_sol, &info,
             &infeasible_constraint);
  printf("info = %d\n", info);
  EXPECT_EQ(info, 1);

  const KinematicsCache<double> cache = model->doKinematics(q_sol);
  const Vector3d com = model->centerOfMass(cache);
  printf("%5.6f\n%5.6f\n%5.6f\n", com(0), com(1), com(2));
  EXPECT_TRUE(
      CompareMatrices(com, Vector3d(0, 0, 1), 1e-6,
                      MatrixCompareType::absolute));
}

GTEST_TEST(testIK, iiwaIK) {
  auto model = std::make_unique<RigidBodyTree<double>>();

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf",
      drake::multibody::joints::kRollPitchYaw, model.get());

  // Create a timespan for the constraints.  It's not particularly
  // meaningful in this test since inverseKin() only tests a single
  // point, but the constructors need something.
  const Vector2d tspan(0, 1);

  // Start the robot in the zero configuration (all joints zeroed,
  // pointing straight up).
  const VectorXd q0 = model->getZeroConfiguration();

  // Constrain iiwa_link_7 (the end effector) to move 0.58 on the X
  // axis and down slightly (to make room for the X axis motion).
  const Vector3d pos_end(0.58, 0, 0.77);
  const double pos_tol = 0.01;
  const Vector3d pos_lb = pos_end - Vector3d::Constant(pos_tol);
  const Vector3d pos_ub = pos_end + Vector3d::Constant(pos_tol);
  const int link_7_idx = model->FindBodyIndex("iiwa_link_7");
  const WorldPositionConstraint wpc(model.get(), link_7_idx,
                              Vector3d::Zero(), pos_lb, pos_ub, tspan);

  // Constrain iiwa_joint_4 between 0.9 and 1.0.
  PostureConstraint pc(model.get(), tspan);
  const drake::Vector1d joint_lb(0.9);
  const drake::Vector1d joint_ub(1.0);

  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) = model->FindChildBodyOfJoint("iiwa_joint_4")->
      get_position_start_index();
  pc.setJointLimits(joint_position_start_idx, joint_lb, joint_ub);

  const std::vector<const RigidBodyConstraint*> constraint_array{&wpc, &pc};
  const IKoptions ikoptions(model.get());

  VectorXd q_sol = VectorXd::Zero(model->get_num_positions());
  int info = 0;
  std::vector<std::string> infeasible_constraint;
  inverseKin(model.get(), q0, q0, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_sol, &info,
             &infeasible_constraint);
  EXPECT_EQ(info, 1);

  // Check that our constrained joint is within where we tried to constrain it.
  EXPECT_GE(q_sol(joint_position_start_idx(0)), joint_lb(0));
  EXPECT_LE(q_sol(joint_position_start_idx(0)), joint_ub(0));

  // Check that the link we were trying to position wound up where we expected.
  const KinematicsCache<double> cache = model->doKinematics(q_sol);
  EXPECT_TRUE(CompareMatrices(
      pos_end, model->relativeTransform(cache, 0, link_7_idx).translation(),
      pos_tol + 1e-6, MatrixCompareType::absolute));
}

}  // namespace
}  // namespace drake
