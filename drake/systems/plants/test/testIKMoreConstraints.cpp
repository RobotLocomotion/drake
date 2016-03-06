#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "../constraint/RigidBodyConstraint.h"
#include "../IKoptions.h"
#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>
#include <numeric>  // for iota

using namespace std;
using namespace Eigen;

// Find the joint position indices corresponding to 'name'
vector<int> getJointPositionVectorIndices(const RigidBodyTree &model,
                                          const std::string &name) {
  shared_ptr<RigidBody> joint_parent_body = model.findJoint(name);
  int num_positions = joint_parent_body->getJoint().getNumPositions();
  vector<int> ret(static_cast<size_t>(num_positions));

  // fill with sequentially increasing values, starting at
  // joint_parent_body->position_num_start:
  iota(ret.begin(), ret.end(), joint_parent_body->position_num_start);
  return ret;
}

void findJointAndInsert(const RigidBodyTree &model, const std::string &name,
                        vector<int> &position_list) {
  auto position_indices = getJointPositionVectorIndices(model, name);

  position_list.insert(position_list.end(), position_indices.begin(),
                       position_indices.end());
}

int main() {
  RigidBodyTree model("examples/Atlas/urdf/atlas_minimal_contact.urdf");

  Vector2d tspan;
  tspan << 0, 1;

  // Default Atlas v5 posture:
  VectorXd qstar(model.num_positions);
  qstar << -0.0260, 0, 0.8440, 0, 0, 0, 0, 0, 0, 0.2700, 0, 0.0550, -0.5700,
      1.1300, -0.5500, -0.0550, -1.3300, 2.1530, 0.5000, 0.0985, 0, 0.0008,
      -0.2700, 0, -0.0550, -0.5700, 1.1300, -0.5500, 0.0550, 1.3300, 2.1530,
      -0.5000, 0.0985, 0, 0.0008, 0.2564;

  // 1 Back Posture Constraint
  PostureConstraint kc_posture_back(&model, tspan);
  std::vector<int> back_idx;
  findJointAndInsert(model, "back_bkz", back_idx);
  findJointAndInsert(model, "back_bky", back_idx);
  findJointAndInsert(model, "back_bkx", back_idx);
  VectorXd back_lb = VectorXd::Zero(3);
  VectorXd back_ub = VectorXd::Zero(3);
  kc_posture_back.setJointLimits(3, back_idx.data(), back_lb, back_ub);

  // 2 Knees Constraint
  PostureConstraint kc_posture_knees(&model, tspan);
  std::vector<int> knee_idx;
  findJointAndInsert(model, "l_leg_kny", knee_idx);
  findJointAndInsert(model, "r_leg_kny", knee_idx);
  VectorXd knee_lb = VectorXd::Zero(2);
  knee_lb(0) = 1.0;  // usually use 0.6
  knee_lb(1) = 1.0;  // usually use 0.6
  VectorXd knee_ub = VectorXd::Zero(2);
  knee_ub(0) = 2.5;
  knee_ub(1) = 2.5;
  kc_posture_knees.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

  // 3 Left Arm Constraint
  PostureConstraint kc_posture_larm(&model, tspan);
  std::vector<int> larm_idx;
  findJointAndInsert(model, "l_arm_shz", larm_idx);
  findJointAndInsert(model, "l_arm_shx", larm_idx);
  findJointAndInsert(model, "l_arm_ely", larm_idx);
  findJointAndInsert(model, "l_arm_elx", larm_idx);
  findJointAndInsert(model, "l_arm_uwy", larm_idx);
  findJointAndInsert(model, "l_arm_mwx", larm_idx);
  findJointAndInsert(model, "l_arm_lwy", larm_idx);
  VectorXd larm_lb = VectorXd::Zero(7);
  larm_lb(0) = 0.27;
  larm_lb(1) = -1.33;
  larm_lb(2) = 2.153;
  larm_lb(3) = 0.500;
  larm_lb(4) = 0.0985;
  larm_lb(5) = 0;
  larm_lb(6) = 0.0008;
  VectorXd larm_ub = larm_lb;
  kc_posture_larm.setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);

  // 4 Left Foot Position and Orientation Constraints
  int l_foot = model.findLinkId("l_foot");
  Vector3d l_foot_pt = Vector3d::Zero();
  Vector3d lfoot_pos0;
  lfoot_pos0(0) = 0;
  lfoot_pos0(1) = 0.13;
  lfoot_pos0(2) = 0.08;
  Vector3d lfoot_pos_lb = lfoot_pos0;
  lfoot_pos_lb(0) += 0.001;
  lfoot_pos_lb(1) += 0.001;
  lfoot_pos_lb(2) += 0.001;
  Vector3d lfoot_pos_ub = lfoot_pos_lb;
  lfoot_pos_ub(2) += 0.01;
  // std::cout << lfoot_pos0.transpose() << " lfoot\n" ;
  WorldPositionConstraint kc_lfoot_pos(&model, l_foot, l_foot_pt, lfoot_pos_lb,
                                       lfoot_pos_ub, tspan);
  Eigen::Vector4d quat_des(1, 0, 0, 0);
  double tol = 0.0017453292519943296;
  WorldQuatConstraint kc_lfoot_quat(&model, l_foot, quat_des, tol, tspan);

  // 5 Right Foot Position and Orientation Constraints
  int r_foot = model.findLinkId("r_foot");
  Vector3d r_foot_pt = Vector3d::Zero();
  Vector3d rfoot_pos0;
  rfoot_pos0(0) = 0;
  rfoot_pos0(1) = -0.13;
  rfoot_pos0(2) = 0.08;
  Vector3d rfoot_pos_lb = rfoot_pos0;
  rfoot_pos_lb(0) += 0.001;
  rfoot_pos_lb(1) += 0.001;
  rfoot_pos_lb(2) += 0.001;
  Vector3d rfoot_pos_ub = rfoot_pos_lb;
  rfoot_pos_ub(2) += 0.001;
  // std::cout << rfoot_pos0.transpose() << " lfoot\n" ;
  WorldPositionConstraint kc_rfoot_pos(&model, r_foot, r_foot_pt, rfoot_pos_lb,
                                       rfoot_pos_ub, tspan);
  WorldQuatConstraint kc_rfoot_quat(&model, r_foot, quat_des, tol, tspan);

  // 6 Right Position Constraints (actual reaching constraint)
  int r_hand = model.findLinkId("r_hand");
  Vector3d r_hand_pt = Vector3d::Zero();
  Vector3d rhand_pos0;
  // Vector3d rhand_pos0 = model.forwardKin(r_hand_pt, r_hand, 0, 0, 0).value();
  rhand_pos0(0) = 0.2;
  rhand_pos0(1) = -0.5;
  rhand_pos0(2) = 0.4;
  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) += 0.05;
  rhand_pos_lb(1) += 0.05;
  rhand_pos_lb(2) += 0.05;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.05;
  // std::cout << rhand_pos_ub.transpose() << " rhand\n" ;
  WorldPositionConstraint kc_rhand(&model, r_hand, r_hand_pt, rhand_pos_lb,
                                   rhand_pos_ub, tspan);

  // 7 QuasiStatic Constraints
  QuasiStaticConstraint kc_quasi(&model, tspan);
  kc_quasi.setShrinkFactor(0.2);
  kc_quasi.setActive(true);
  Eigen::Matrix3Xd l_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
  l_foot_pts << -0.0820, -0.0820, 0.1780, 0.1780, 0.0624, -0.0624, 0.0624,
      -0.0624, -0.0811, -0.0811, -0.0811, -0.0811;
  kc_quasi.addContact(1, &l_foot, &l_foot_pts);
  Eigen::Matrix3Xd r_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
  r_foot_pts << -0.0820, -0.0820, 0.1780, 0.1780, 0.0624, -0.0624, 0.0624,
      -0.0624, -0.0811, -0.0811, -0.0811, -0.0811;
  kc_quasi.addContact(1, &r_foot, &r_foot_pts);

  std::vector<RigidBodyConstraint *> constraint_array;
  constraint_array.push_back(&kc_quasi);
  constraint_array.push_back(&kc_posture_knees);
  constraint_array.push_back(&kc_lfoot_pos);
  constraint_array.push_back(&kc_lfoot_quat);
  constraint_array.push_back(&kc_rfoot_pos);
  constraint_array.push_back(&kc_rfoot_quat);
  constraint_array.push_back(&kc_rhand);
  constraint_array.push_back(&kc_posture_larm);
  constraint_array.push_back(&kc_posture_back);

  IKoptions ikoptions(&model);
  VectorXd q_sol(model.num_positions);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(&model, qstar, qstar, constraint_array.size(),
             constraint_array.data(), q_sol, info, infeasible_constraint,
             ikoptions);
  printf("INFO = %d\n", info);
  if (info != 1) {
    return 1;
  }

  /////////////////////////////////////////
  KinematicsCache<double> cache = model.doKinematics(q_sol);
  Vector3d com = model.centerOfMass(cache);
  printf("%5.2f\n%5.2f\n%5.2f\n", com(0), com(1), com(2));

  return 0;
}
