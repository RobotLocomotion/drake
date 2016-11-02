#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::GetDrakePath;

GTEST_TEST(testIKtraj, testIKtraj) {
  RigidBodyTree model(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_minimal_contact.urdf");

  int r_hand{};
  int pelvis{};
  for (int i = 0; i < static_cast<int>(model.bodies.size()); i++) {
    if (!model.bodies[i]->get_name().compare(std::string("r_hand"))) {
      r_hand = i;
    }
    if (!model.bodies[i]->get_name().compare(std::string("pelvis"))) {
      pelvis = i;
    }
  }

  VectorXd qstar = model.getZeroConfiguration();
  qstar(3) = 0.8;
  KinematicsCache<double> cache = model.doKinematics(qstar);
  Vector3d com0 = model.centerOfMass(cache);

  Vector3d r_hand_pt = Vector3d::Zero();
  Vector3d rhand_pos0 = model.transformPoints(cache, r_hand_pt, r_hand, 0);

  Vector2d tspan(0, 1);
  int nT = 4;
  double dt = tspan(1) / (nT - 1);
  std::vector<double> t(nT, 0);
  for (int i = 0; i < nT; i++) {
    t[i] = dt * i;
  }
  MatrixXd q0 = qstar.replicate(1, nT);
  VectorXd qdot0 = VectorXd::Zero(model.get_num_velocities());
  Vector3d com_lb = com0;
  com_lb(0) = std::numeric_limits<double>::quiet_NaN();
  com_lb(1) = std::numeric_limits<double>::quiet_NaN();
  Vector3d com_ub = com0;
  com_ub(0) = std::numeric_limits<double>::quiet_NaN();
  com_ub(1) = std::numeric_limits<double>::quiet_NaN();
  com_ub(2) = com0(2) + 0.5;
  WorldCoMConstraint com_kc(&model, com_lb, com_ub);
  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) += 0.1;
  rhand_pos_lb(1) += 0.05;
  rhand_pos_lb(2) += 0.25;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.25;
  Vector2d tspan_end;
  tspan_end << t[nT - 1], t[nT - 1];
  WorldPositionConstraint kc_rhand(
      &model, r_hand, r_hand_pt, rhand_pos_lb, rhand_pos_ub, tspan_end);

  // Add a multiple time constraint which is fairly trivial to meet in
  // this case.
  WorldFixedBodyPoseConstraint kc_fixed_pose(&model, pelvis, tspan);

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&com_kc);
  constraint_array.push_back(&kc_rhand);
  constraint_array.push_back(&kc_fixed_pose);

  IKoptions ikoptions(&model);
  MatrixXd q_sol(model.get_num_positions(), nT);
  MatrixXd qdot_sol(model.get_num_velocities(), nT);
  MatrixXd qddot_sol(model.get_num_positions(), nT);
  int info = 0;
  std::vector<std::string> infeasible_constraint;

  inverseKinTraj(&model, nT, t.data(), qdot0, q0, q0, constraint_array.size(),
                 constraint_array.data(), ikoptions, &q_sol, &qdot_sol,
                 &qddot_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);

  ikoptions.setFixInitialState(false);
  ikoptions.setMajorIterationsLimit(500);

  inverseKinTraj(&model, nT, t.data(), qdot0, q0, q0, constraint_array.size(),
                 constraint_array.data(), ikoptions, &q_sol, &qdot_sol,
                 &qddot_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);

  Eigen::RowVectorXd t_inbetween(5);
  t_inbetween << 0.1, 0.15, 0.3, 0.4, 0.6;
  ikoptions.setAdditionaltSamples(t_inbetween);
  inverseKinTraj(&model, nT, t.data(), qdot0, q0, q0, constraint_array.size(),
                 constraint_array.data(), ikoptions, &q_sol, &qdot_sol,
                 &qddot_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, 1);
}
