// finds a standing posture of Valkyrie by solving IK and displays it in drake
// visualizer.

#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include "gtest/gtest.h"

// Includes for IK solver.
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/inverse_kinematics_backend.h"
#include "drake/multibody/rigid_body_ik.h"

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"
#include "drake/systems/framework/primitives/trajectory_source.h"
#include "drake/systems/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/util/convexHull.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;

namespace drake {
namespace examples {
namespace valkyrie {
namespace {

/* Finds and returns the indices within the state vector of @p tree that contain
 * the position states of a joint named @p name. The model instance ID is
 * ignored in this search (joints belonging to all model instances are
 * searched).
 */
std::vector<int> GetJointPositionVectorIndices(const RigidBodyTreed* tree,
                                               const std::string& name) {
  RigidBody<double>* joint_child_body = tree->FindChildBodyOfJoint(name);
  int num_positions = joint_child_body->getJoint().get_num_positions();
  std::vector<int> ret(static_cast<size_t>(num_positions));

  // Since the joint position states are located in a contiguous region of the
  // the rigid body tree's state vector, fill the return vector with
  // sequentially increasing indices starting at
  // `joint_child_body->get_position_start_index()`.
  std::iota(ret.begin(), ret.end(),
            joint_child_body->get_position_start_index());
  return ret;
}

/* Finds the indices within the state vector of @p tree that contains the
 * position states of a joint named @p name, and appends the vector of
 * indices found to the end of @p position_list.
 */
void FindJointAndInsert(const RigidBodyTreed* model, const std::string& name,
                        std::vector<int>* const position_list) {
  auto position_indices = GetJointPositionVectorIndices(model, name);

  position_list->insert(position_list->end(), position_indices.begin(),
                        position_indices.end());
}

Vector3d FindCenter(const Eigen::Matrix3Xd& vertices) {
  Vector3d center;
  center.setZero();
  for (int i = 0; i < vertices.cols(); i++) {
    center += vertices.col(i);
  }
  return center / vertices.cols();
}

bool IsInsideFeet(const Eigen::Matrix3Xd& vertices, const Vector3d& CoM) {
  // vertices = [l_hand[1], rhand[1], l_foot[4], r_foot[4]]
  Eigen::MatrixXd contact_pts = vertices.rightCols(8);
  const double shrink_factor = 0.9;
  Vector3d center_pos = FindCenter(contact_pts);
  int n_pts = contact_pts.cols();
  Eigen::MatrixXd support_pts(2, n_pts);
  for (int i = 0; i < n_pts; i++) {
    support_pts.col(i) = center_pos.head(2) * (1.0 - shrink_factor) +
                         contact_pts.block(0, i, 2, 1) * shrink_factor;
  }
  return inConvexHull(support_pts, CoM.head(2), 1e-8);
}

void TransformPtsBatch(const RigidBodyTreed* tree,
                       const KinematicsCache<double>& cache,
                       const Eigen::Matrix3Xd& pts,
                       const std::vector<int>& bodies, Matrix3Xd* pts_world) {
  for (int j = 0; j < pts.cols(); j++) {
    pts_world->col(j) = tree->transformPoints(cache, pts.col(j), bodies[j], 0);
  }
}

Vector2d convex_combination(Vector2d x, Vector2d y, double lambda) {
  return x + lambda * (y - x);
}

int DoMain() {
  std::shared_ptr<RigidBodyTreed> tree = std::make_shared<RigidBodyTreed>(
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_sim_drake_one_neck_dof_additional_contact_pts.urdf",
      drake::multibody::joints::kRollPitchYaw);

  for (int i = 0; i < tree->get_num_bodies(); i++)
    std::cout << i << " " << tree->getBodyOrFrameName(i) << std::endl;

  VectorXd standing_pose(tree->get_num_positions());
  standing_pose << 0.0,  // base_x
      0.0,               // base_y
      1.025,             // base_z
      0.0,               // base_roll
      0.0,               // base_pitch
      0.0,               // 5 base_yaw
      0.0,               // 6 torsoYaw
      0.0,               // 7 torsoPitch
      0.0,               // 8 torsoRoll
      0.0,               // 9 lowerNeckPitch
      // 0.0,                  // 10 neckYaw
      // 0.0,                  // 11 upperNeckPitch

      0.30019663134302466,  // 12 rightShoulderPitch
      1.25,                 // 13 rightShoulderRoll
      0.0,                  // 14 rightShoulderYaw
      0.7853981633974483,   // 15 rightElbowPitch
      1.571,                // 16 rightForearmYaw
      0.0,                  // 17 rightWristRoll
      0.0,                  // 18 rightWristPItch

      0.30019663134302466,  // 19 leftShoulderPitch
      -1.25,                // 20 leftShoulderRoll
      0.0,                  // 21 leftShoulderYaw
      -0.7853981633974483,  // 22 leftElbowPitch
      1.571,                // 23 leftForearmYaw
      0.0,                  // 24 leftWristRoll
      0.0,                  // 25 LeftWristPitch

      0.0,    // 26 rightHipYaw
      0.0,    // 27 rightHipRoll
      -0.49,  // 28 rightHipPitch
      1.205,  // 29 rightKneePitch
      -0.71,  // 30 rightAnklePitch
      0.0,    // 31 rightAnkleRoll

      0.0,    // 32 leftHipYaw
      0.0,    // 33 leftHipRoll
      -0.49,  // 34 leftHipPitch
      1.205,  // 35 leftKneePitch
      -0.71,  // 36 leftAnklePitch
      0.0;    // 37 leftAnkleRoll

  // the robot could be floating in this pose
  VectorXd prone_pose(tree->get_num_positions());
  prone_pose << 0.7,  // base_x
      0.0,            // base_y
      0.5,            // base_z
      0.0,            // base_roll
      2.2,            // base_pitch
      0.0,            // 5 base_yaw
      0.0,            // 6 torsoYaw
      0.0,            // 7 torsoPitch
      0.0,            // 8 torsoRoll
      0.0,            // 9 lowerNeckPitch
      // 0.0,                  // 10 neckYaw
      // 0.0,                  // 11 upperNeckPitch

      -120.0 / 180 * M_PI,  // 12 rightShoulderPitch [-130, -40]
      80.0 / 180 * M_PI,    // 13 rightShoulderRoll  [70, 85]
      0.05,                 // 14 rightShoulderYaw [fixed]
      0.0,                  // 15 rightElbowPitch [-6, 120]
      0.0,                  // 16 rightForearmYaw [fixed]
      0.0,                  // 17 rightWristRoll [fixed]
      -0.49,                // 18 rightWristPitch [fixed]

      -120.0 / 180 * M_PI,  // 19 leftShoulderPitch [-130 -40]
      -80.0 / 180 * M_PI,   // 20 leftShoulderRoll [-85, -70]
      -0.05,                // 21 leftShoulderYaw
      0.0,                  // 22 leftElbowPitch [-120, 6]
      0.0,                  // 23 leftForearmYaw
      0.0,                  // 24 leftWristRoll
      0.49,                 // 25 LeftWristPitch

      0.0,      // 26 rightHipYaw
      0.0,      // 27 rightHipRoll
      -1.74,    // 28 rightHipPitch
      0.6,      // 29 rightKneePitch
      -0.8644,  // 30 rightAnklePitch
      0.0,      // 31 rightAnkleRoll

      0.0,      // 32 leftHipYaw
      0.0,      // 33 leftHipRoll
      -1.74,    // 34 leftHipPitch
      0.6,      // 35 leftKneePitch
      -0.8644,  // 36 leftAnklePitch
      0.0;      // 37 leftAnkleRoll

  // the robot's hands and feet are on the ground
  VectorXd prone_pose_2(tree->get_num_positions());
  prone_pose_2 << 0.785523,  // base_x
      0.0,                   // base_y
      0.711749,              // base_z
      -0.00164626,           // base_roll
      1.91036,               // base_pitch
      0.00549406,            // 5 base_yaw
      -0.0153586,            // 6 torsoYaw
      -0.122173,             // 7 torsoPitch
      -0.0115514,            // 8 torsoRoll
      0.0,                   // 9 lowerNeckPitch
      // 0.0,                  // 10 neckYaw
      // 0.0,                  // 11 upperNeckPitch

      -2.0944,  // 12 rightShoulderPitch [-130, -40]
      1.39626,  // 13 rightShoulderRoll  [70, 85]
      0.05,     // 14 rightShoulderYaw [fixed]
      0.0,      // 15 rightElbowPitch [-6, 120]
      0.0,      // 16 rightForearmYaw [fixed]
      0.0,      // 17 rightWristRoll [fixed]
      -0.49,    // 18 rightWristPitch [fixed]

      -2.11865,     // 19 leftShoulderPitch [-130 -40]
      -1.37398,     // 20 leftShoulderRoll [-85, -70]
      -0.05,        // 21 leftShoulderYaw
      -0.00917247,  // 22 leftElbowPitch [-120, 6]
      0.0,          // 23 leftForearmYaw
      0.0,          // 24 leftWristRoll
      0.49,         // 25 LeftWristPitch

      0.00469986,   // 26 rightHipYaw
      0.00431728,   // 27 rightHipRoll
      -1.73985,     // 28 rightHipPitch
      0.614187,     // 29 rightKneePitch
      -0.79335,     // 30 rightAnklePitch
      -0.00354192,  // 31 rightAnkleRoll

      -0.00148586,  // 32 leftHipYaw
      0.00663385,   // 33 leftHipRoll
      -1.74006,     // 34 leftHipPitch
      0.614074,     // 35 leftKneePitch
      -0.792986,    // 36 leftAnklePitch
      0.00306235;   // 37 leftAnkleRoll

  // a prone pose where only toes are contacting the ground
  VectorXd prone_pose_toes(tree->get_num_positions());
  prone_pose_toes << 0.0,  // base_x
      0.0,                 // base_y
      0.441116,            // base_z
      0.0,                 // base_roll
      1.21367,             // base_pitch
      0.0,                 // 5 base_yaw
      0.0,                 // 6 torsoYaw
      0.139111,            // 7 torsoPitch
      0.0,                 // 8 torsoRoll
      0.0,                 // 9 lowerNeckPitch
      // 0.0,                  // 10 neckYaw
      // 0.0,                  // 11 upperNeckPitch

      -2.0944,  // 12 rightShoulderPitch [-130, -40]
      1.39626,  // 13 rightShoulderRoll  [70, 85]
      0.05,     // 14 rightShoulderYaw [fixed]
      0.0,      // 15 rightElbowPitch [-6, 120]
      0.0,      // 16 rightForearmYaw [fixed]
      0.0,      // 17 rightWristRoll [fixed]
      -0.49,    // 18 rightWristPitch [fixed]

      -2.06676,  // 19 leftShoulderPitch [-130 -40]
      -1.40269,  // 20 leftShoulderRoll [-85, -70]
      -0.05,     // 21 leftShoulderYaw
      0.013267,  // 22 leftElbowPitch [-120, 6]
      0,         // 23 leftForearmYaw
      0,         // 24 leftWristRoll
      0.49,      // 25 LeftWristPitch

      0.0,    // 26 rightHipYaw
      0.0,    // 27 rightHipRoll
      -0.48,  // 28 rightHipPitch
      1.21,   // 29 rightKneePitch
      -0.71,  // 30 rightAnklePitch
      0.0,    // 31 rightAnkleRoll

      0.0,    // 32 leftHipYaw
      0.0,    // 33 leftHipRoll
      -0.48,  // 34 leftHipPitch
      1.21,   // 35 leftKneePitch
      -0.71,  // 36 leftAnklePitch
      0.0;    // 37 leftAnkleRoll

  VectorXd squat_pose(tree->get_num_positions());
  squat_pose << -0.141714353,  // 0
      0.157190702,             // 1
      0.665432187,             // 2
      -0.895860474000000,      // 3
      0.829941870000000,       // 4
      -1.15801324000000,       // 5
      -0.146479404000000,      // 6
      0.192227111000000,       // 7
      -0.143599135000000,      // 8
      0,                       // 9
      // 0, //10
      // 0, //11
      -0.0644158134000000,   // 12
      1.10416339000000,      // 13
      -0.00726552442000000,  // 14
      0.240364862000000,     // 15
      2.12441686000000,      // 16
      0.155591530000000,     // 17
      0.0307877965000000,    // 18
      -1.92824968000000,     // 19
      -0.618868790000000,    // 20
      -0.152318189000000,    // 21
      -0.213302160000000,    // 22
      3.14000000000000,      // 23
      0.00160567432000000,   // 24
      0.490000000000000,     // 25

      0.345962604000000,   // 26
      -0.249349812000000,  // 27
      -2.37711866000000,   // 28
      1.15699985000000,    // 29
      0.0306223996000000,  // 30
      0.348000000000000,   // 31

      0.964904798000000,   // 32
      0.0342466052000000,  // 33
      -2.42000000000000,   // 34
      1.90000000000000,    // 35
      -0.595530575000000,  // 36
      -0.324304201000000;  // 37

  // a squat pose with both hands on the ground
  VectorXd squat_pose_2(tree->get_num_positions());
  squat_pose_2 << 0.714247,  // base_x
      0.0,                    // base_y
      0.666345,               // base_z
      -0.914722,              // base_roll
      0.994925,               // base_pitch
      -1.12529,               // 5 base_yaw

      0.366227,   // 6 torsoYaw
      0.495029,   // 7 torsoPitch
      -0.226893,  // 8 torsoRoll
      0.0,        // 9 lowerNeckPitch
      // 0.0,                  // 10 neckYaw
      // 0.0,                  // 11 upperNeckPitch

      -2.0944,  // 12 rightShoulderPitch
      1.39626,  // 13 rightShoulderRoll
      0.05,     // 14 rightShoulderYaw
      0.0,      // 15 rightElbowPitch
      0.0,      // 16 rightForearmYaw
      0.0,      // 17 rightWristRoll
      -0.49,    // 18 rightWristPItch

      -2.04374,   // 19 leftShoulderPitch
      -1.22173,   // 20 leftShoulderRoll
      -0.05,      // 21 leftShoulderYaw
      -0.286651,  // 22 leftElbowPitch
      0.0,        // 23 leftForearmYaw
      0.0,        // 24 leftWristRoll
      0.49,       // 25 LeftWristPitch

      0.196557,     // 26 rightHipYaw
      -0.220605,    // 27 rightHipRoll
      -2.42,        // 28 rightHipPitch
      1.12927,      // 29 rightKneePitch
      -0.00258235,  // 30 rightAnklePitch
      0.331879,     // 31 rightAnkleRoll

      0.851394,    // 32 leftHipYaw
      0.00531158,  // 33 leftHipRoll
      -2.37968,    // 34 leftHipPitch
      1.79068,     // 35 leftKneePitch
      -0.612404,   // 36 leftAnklePitch
      -0.354857;   // 37 leftAnkleRoll

  // Setting up constraints, based on testIKMoreConstraints.cpp and
  // director-generated M-file.
  const double inf = std::numeric_limits<double>::infinity();

  Vector2d tspan(0, 1);
  // Neck Posture Constraint, posture constraints are imposed on q
  PostureConstraint kc_posture_neck(tree.get(), tspan);
  std::vector<int> neck_idx;
  FindJointAndInsert(tree.get(), "lowerNeckPitch", &neck_idx);
  FindJointAndInsert(tree.get(), "neckYaw", &neck_idx);
  FindJointAndInsert(tree.get(), "upperNeckPitch", &neck_idx);
  VectorXd neck_lb = VectorXd::Zero(neck_idx.size());
  VectorXd neck_ub = VectorXd::Zero(neck_idx.size());
  kc_posture_neck.setJointLimits(neck_idx.size(), neck_idx.data(), neck_lb,
                                 neck_ub);

  // no collision constraint
  std::vector<int> active_body_idx;
  // active_body_idx.push_back(l_foot);
  // active_body_idx.push_back(r_foot);
  std::set<std::string> active_group_names;
  active_group_names.insert("l_leg");
  active_group_names.insert("r_leg");
  active_group_names.insert("l_arm");
  active_group_names.insert("r_arm");
  active_group_names.insert("l_uleg");
  active_group_names.insert("r_uleg");
  active_group_names.insert("core");
  AllBodiesClosestDistanceConstraint no_collision(
      tree.get(), 0.0, inf, active_body_idx, active_group_names, tspan);

  // Torso posture constraint
  PostureConstraint kc_posture_torso(tree.get(), tspan);
  std::vector<int> torso_idx;
  FindJointAndInsert(tree.get(), "torsoYaw", &torso_idx);
  FindJointAndInsert(tree.get(), "torsoPitch", &torso_idx);
  FindJointAndInsert(tree.get(), "torsoRoll", &torso_idx);
  /*
  Vector3d torso_nominal = Vector3d::Zero();
  Vector3d torso_half_range(75.0 / 180 * M_PI, 60.0 / 180 * M_PI,
                            60.0 / 180 * M_PI);
  Vector3d torso_lb_relaxed = torso_nominal - torso_half_range;
  Vector3d torso_ub_relaxed = torso_nominal + torso_half_range;
  torso_lb_relaxed(1) = -5.0 / 180 * M_PI;
  */
  // bounds as defined in URDF file
  Vector3d torso_lb(-75.0 / 180 * M_PI, -7.0 / 180 * M_PI, -13.0 / 180 * M_PI);
  Vector3d torso_ub(65.0 / 180 * M_PI, 38.0 / 180 * M_PI, 13.0 / 180 * M_PI);
  kc_posture_torso.setJointLimits(3, torso_idx.data(), torso_lb, torso_ub);

  // knee posture constraint
  PostureConstraint kc_posture_knee(tree.get(), tspan);
  std::vector<int> knee_idx;
  FindJointAndInsert(tree.get(), "leftKneePitch", &knee_idx);
  FindJointAndInsert(tree.get(), "rightKneePitch", &knee_idx);
  Vector2d knee_lb(-4.0 / 180 * M_PI, -4.0 / 180 * M_PI);
  Vector2d knee_ub(115.0 / 180 * M_PI, 115.0 / 180 * M_PI);
  kc_posture_knee.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

  // Left foot position and orientation constraint, position and orientation
  // constraints are imposed on frames/bodies
  Vector2d tspan_foot_flat(0.5, 1);
  const Vector3d origin(0, 0, 0);

  auto leftFootPtr = tree->FindBody("leftFoot");
  Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
  Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
  Matrix3Xd l_foot_toes = l_foot_pts.rightCols(2);
  std::cout << "left foot contact pts: " << std::endl;
  std::cout << l_foot_pts << std::endl;

  int l_foot = tree->FindBodyIndex("leftFoot");
  Vector4d lfoot_quat(1, 0, 0, 0);
  Vector3d lfoot_pos_lb_one_pt(-inf, -inf, 0);
  Matrix3Xd lfoot_pos_lb = lfoot_pos_lb_one_pt.replicate(1, 2);
  // Position and quaternion constraints are relaxed to make the problem
  // solvable by IPOPT.
  lfoot_pos_lb(2, 0) -= 0.0001;
  lfoot_pos_lb(2, 1) -= 0.0001;
  std::cout << "lfoot_pos_lb: " << std::endl << lfoot_pos_lb << std::endl;
  Vector3d lfoot_pos_ub_one_pt(inf, inf, 0);
  Matrix3Xd lfoot_pos_ub = lfoot_pos_ub_one_pt.replicate(1, 2);
  lfoot_pos_ub(2, 0) += 0.0001;
  lfoot_pos_ub(2, 1) += 0.0001;
  std::cout << "lfoot_pos_ub: " << std::endl << lfoot_pos_ub << std::endl;
  WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, l_foot_toes,
                                       lfoot_pos_lb, lfoot_pos_ub, tspan);
  double tol = 0.5 / 180 * M_PI;
  WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol,
                                    tspan_foot_flat);

  // Right foot position and orientation constraint
  auto rightFootPtr = tree->FindBody("rightFoot");
  Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
  Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
  Matrix3Xd r_foot_toes = r_foot_pts.rightCols(2);
  std::cout << "right foot contact pts: " << std::endl;
  std::cout << r_foot_pts << std::endl;

  int r_foot = tree->FindBodyIndex("rightFoot");
  Vector4d rfoot_quat(1, 0, 0, 0);
  WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, r_foot_toes,
                                       lfoot_pos_lb, lfoot_pos_ub, tspan);
  WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol,
                                    tspan_foot_flat);

  // Pelvis height constraint
  int pelvis = tree->FindBodyIndex("Pelvis");
  Vector3d pelvis_pos_lb(-inf, -inf, 0);
  Vector3d pelvis_pos_ub(inf, inf, 0.5);
  WorldPositionConstraint kc_pelvis_pos(tree.get(), pelvis, origin,
                                        pelvis_pos_lb, pelvis_pos_ub, tspan);

  // Left arm posture constraint
  // Vector2d tspan_arm_posture(0, 2.5);
  Vector2d tspan_arm_posture(0, 1);
  PostureConstraint kc_posture_larm(tree.get(), tspan_arm_posture);
  std::vector<int> larm_idx;
  FindJointAndInsert(tree.get(), "leftShoulderPitch", &larm_idx);
  FindJointAndInsert(tree.get(), "leftShoulderRoll", &larm_idx);
  FindJointAndInsert(tree.get(), "leftShoulderYaw", &larm_idx);
  FindJointAndInsert(tree.get(), "leftElbowPitch", &larm_idx);
  FindJointAndInsert(tree.get(), "leftForearmYaw", &larm_idx);
  FindJointAndInsert(tree.get(), "leftWristRoll", &larm_idx);
  FindJointAndInsert(tree.get(), "leftWristPitch", &larm_idx);
  Eigen::Matrix<double, 7, 1> larm_nominal;
  larm_nominal.setZero();
  for (int i = 0; i < 7; i++) larm_nominal(i) = prone_pose(larm_idx[i]);
  Eigen::Matrix<double, 7, 1> larm_lb = larm_nominal;
  Eigen::Matrix<double, 7, 1> larm_ub = larm_nominal;
  larm_ub[0] = -40.0 / 180 * M_PI;
  larm_ub[1] = -70.0 / 180 * M_PI;
  larm_ub[3] = 6.0 / 180 * M_PI;

  larm_lb[0] = -130.0 / 180 * M_PI;
  larm_lb[1] = -85.0 / 180 * M_PI;
  larm_lb[3] = -120.0 / 180 * M_PI;

  kc_posture_larm.setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);

  // Right arm posture constraint
  PostureConstraint kc_posture_rarm(tree.get(), tspan_arm_posture);
  std::vector<int> rarm_idx;
  FindJointAndInsert(tree.get(), "rightShoulderPitch", &rarm_idx);
  FindJointAndInsert(tree.get(), "rightShoulderRoll", &rarm_idx);
  FindJointAndInsert(tree.get(), "rightShoulderYaw", &rarm_idx);
  FindJointAndInsert(tree.get(), "rightElbowPitch", &rarm_idx);
  FindJointAndInsert(tree.get(), "rightForearmYaw", &rarm_idx);
  FindJointAndInsert(tree.get(), "rightWristRoll", &rarm_idx);
  FindJointAndInsert(tree.get(), "rightWristPitch", &rarm_idx);
  Eigen::Matrix<double, 7, 1> rarm_nominal;
  rarm_nominal.setZero();
  for (int i = 0; i < 7; i++) rarm_nominal(i) = prone_pose(rarm_idx[i]);
  Eigen::Matrix<double, 7, 1> rarm_lb = rarm_nominal;
  Eigen::Matrix<double, 7, 1> rarm_ub = rarm_nominal;
  larm_ub[0] = -40.0 / 180 * M_PI;
  larm_ub[1] = 85.0 / 180 * M_PI;
  larm_ub[3] = 120.0 / 180 * M_PI;

  larm_lb[0] = -130.0 / 180 * M_PI;
  larm_lb[1] = 70.0 / 180 * M_PI;
  larm_lb[3] = -6.0 / 180 * M_PI;

  kc_posture_rarm.setJointLimits(7, rarm_idx.data(), rarm_lb, rarm_ub);

  // 8 Quasistatic constraint
  Vector2d tspan_4pts(0, 1);
  Vector2d tspan_3pts(1.5, 2.5);
  Vector2d tspan_2pts(2.5, 3);
  QuasiStaticConstraint kc_kuasi_4pts(tree.get(), tspan_4pts);
  QuasiStaticConstraint kc_kuasi_3pts(tree.get(), tspan_3pts);
  QuasiStaticConstraint kc_kuasi_2pts(tree.get(), tspan_2pts);
  kc_kuasi_4pts.setShrinkFactor(0.9);
  kc_kuasi_4pts.setActive(true);
  kc_kuasi_3pts.setShrinkFactor(0.9);
  kc_kuasi_3pts.setActive(true);
  kc_kuasi_2pts.setShrinkFactor(0.9);
  kc_kuasi_2pts.setActive(true);

  kc_kuasi_4pts.addContact(1, &l_foot, &l_foot_pts);
  kc_kuasi_3pts.addContact(1, &l_foot, &l_foot_pts);
  kc_kuasi_2pts.addContact(1, &l_foot, &l_foot_pts);

  kc_kuasi_4pts.addContact(1, &r_foot, &r_foot_pts);
  kc_kuasi_3pts.addContact(1, &r_foot, &r_foot_pts);
  kc_kuasi_2pts.addContact(1, &r_foot, &r_foot_pts);

  auto leftArmPtr = tree->FindBody("leftForearmLink");
  Matrix3Xd leftArmContactPts = leftArmPtr->get_contact_points();
  std::cout << "left arm contact pts: " << std::endl
            << leftArmContactPts << std::endl;
  const int l_forearm = tree->FindBodyIndex("leftForearmLink");
  kc_kuasi_4pts.addContact(1, &l_forearm, &leftArmContactPts);
  kc_kuasi_3pts.addContact(1, &l_forearm, &leftArmContactPts);

  auto rightArmPtr = tree->FindBody("rightForearmLink");
  Matrix3Xd rightArmContactPts = rightArmPtr->get_contact_points();
  std::cout << "right arm contact pts: " << std::endl
            << rightArmContactPts << std::endl;
  const int r_forearm = tree->FindBodyIndex("rightForearmLink");
  kc_kuasi_4pts.addContact(1, &r_forearm, &rightArmContactPts);

  // 9 leftForeArm position constraint
  Vector2d tspan_larm_sliding(0, 1);
  Vector3d lforearm_pos_lb(1.30, 0.38, 0);
  Vector3d lforearm_pos_ub(1.30, 0.38, 0);
  WorldPositionConstraint kc_lforearm_pos(tree.get(), l_forearm,
                                          leftArmContactPts, lforearm_pos_lb,
                                          lforearm_pos_ub, tspan_larm_sliding);

  // 9 rightForeArm position constraint
  Vector2d tspan_rarm_sliding(0, 1);
  Vector3d rforearm_pos_lb(1.30, -0.38, 0);
  Vector3d rforearm_pos_ub(1.30, -0.38, 0);
  WorldPositionConstraint kc_rforearm_pos(tree.get(), r_forearm,
                                          rightArmContactPts, rforearm_pos_lb,
                                          rforearm_pos_ub, tspan_rarm_sliding);

  // -----------------solve-----------------------------------------------------
  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&kc_posture_neck);
  constraint_array.push_back(&no_collision);
  constraint_array.push_back(&kc_posture_torso);
  constraint_array.push_back(&kc_posture_knee);
  constraint_array.push_back(&kc_lfoot_quat);
  constraint_array.push_back(&kc_rfoot_quat);

  constraint_array.push_back(&kc_posture_larm);
  constraint_array.push_back(&kc_posture_rarm);

  constraint_array.push_back(&kc_lforearm_pos);
  constraint_array.push_back(&kc_rforearm_pos);

  constraint_array.push_back(&kc_lfoot_pos);
  constraint_array.push_back(&kc_rfoot_pos);
  constraint_array.push_back(&kc_kuasi_4pts);

  // constraint_array.push_back(&kc_kuasi_2pts);
  // constraint_array.push_back(&kc_kuasi_3pts);
  // constraint_array.push_back(&kc_pelvis_pos);

  // setting Q
  std::cout << "postions and names: " << std::endl;
  for (int i = 0; i < tree->get_num_positions(); i++)
    std::cout << i << " " << tree->get_position_name(i) << std::endl;

  VectorXd cost(tree->get_num_positions());
  cost.setOnes();

  const int left_hip_pitch_idx = 32;
  const int right_hip_pitch_idx = 26;
  const int left_knee_pitch_idx = 33;
  const int right_knee_pitch_idx = 27;
  const int left_ankle_pitch_idx = 34;
  const int right_ankle_pitch_idx = 28;

  std::vector<int> lower_body_pitch_idx;
  lower_body_pitch_idx.push_back(left_hip_pitch_idx);
  lower_body_pitch_idx.push_back(right_hip_pitch_idx);
  lower_body_pitch_idx.push_back(left_knee_pitch_idx);
  lower_body_pitch_idx.push_back(right_knee_pitch_idx);
  lower_body_pitch_idx.push_back(left_ankle_pitch_idx);
  lower_body_pitch_idx.push_back(right_ankle_pitch_idx);

  std::vector<int> base_translation_idx = {0, 1, 2};
  std::vector<int> base_rotation_idx = {3, 4, 5};
  std::vector<int> hips_idx={26,27,28,32,33,34};
  std::vector<int> legs_idx={29,30,31,35,36,37};
  std::vector<int> arms_idx;
  for (int i = 12 - 2; i <= 25 - 2; i++) arms_idx.push_back(i);
  for(int i=0;i<hips_idx.size();i++) hips_idx[i]-=2;
  for(int i=0;i<legs_idx.size();i++) legs_idx[i]-=2;

  for (const int& i : base_translation_idx) cost(i) = 0;
  for (const int& i : base_rotation_idx) cost(i) = 1e3;
  for (const int& i : torso_idx) cost(i) = 10;
  for (const int& i : legs_idx) cost(i) = 1;
  for (const int& i : hips_idx) cost(i) = 10;
  cost(9) = 1e6;  // neck cost

  // display all costs
  std::cout << "costs are:" << std::endl;
  for (int i = 0; i < cost.size(); i++) {
    std::cout << cost(i) << " " << tree->get_position_name(i) << " " << i
              << std::endl;
  }

  Eigen::MatrixXd Q = cost.asDiagonal();
  Eigen::MatrixXd Qv = 0.05 * Q;
  Eigen::MatrixXd Qa = 0.05 * Q;

  IKoptions ikoptions(tree.get());
  ikoptions.setQ(Q);
  ikoptions.setQa(Qa);
  ikoptions.setQv(Qv);
  ikoptions.setSequentialSeedFlag(true);

  const int kN_per_step = 2;
  int nT1 = kN_per_step + 1;
  std::vector<double> t1;
  for (int i = 0; i < nT1; i++) t1.push_back(1.0 / kN_per_step * i);
  Eigen::MatrixXd q_sol1(tree->get_num_positions(), nT1);
  Eigen::MatrixXd q_nom1(tree->get_num_positions(), nT1);
  q_nom1 = prone_pose_2.replicate(1, nT1);
  std::vector<int> info_v(nT1, 0);
  std::vector<std::string> infeasible_constraint;
  inverseKinPointwise(tree.get(), nT1, t1.data(), q_nom1, q_nom1,
                      constraint_array.size(), constraint_array.data(),
                      ikoptions, &q_sol1, info_v.data(),
                      &infeasible_constraint);
  for (int i = 0; i < info_v.size(); i++) {
    std::cout << "info " << i << ": " << info_v[i] << std::endl;
  }
  /*
  Vector2d x1(0,0);
  Vector2d x2(1,1);
  cout << "convex combination of x and y: " << convex_combination(x1,x2,0.8) <<
       endl;
  */
  KinematicsCache<double> cache_start = tree->doKinematics(prone_pose_2);
  Vector3d left_start = tree->transformPoints(cache_start, origin, l_foot, 0);
  Vector3d right_start = tree->transformPoints(cache_start, origin, r_foot, 0);
  cout << "left start" << endl << left_start << endl;
  cout << "right start" << endl << right_start << endl;
  KinematicsCache<double> cache_end = tree->doKinematics(squat_pose_2);
  Vector3d left_end = tree->transformPoints(cache_end, origin, l_foot, 0);
  Vector3d right_end = tree->transformPoints(cache_end, origin, r_foot, 0);
  cout << "left end" << endl << left_end << endl;
  cout << "right end" << endl << right_end << endl;

  Matrix3Xd contact_pts(3, 10);
  contact_pts << leftArmContactPts, rightArmContactPts, l_foot_pts.rightCols(4),
      r_foot_pts.rightCols(4);
  const std::vector<int> bodies = {l_forearm, r_forearm, l_foot, l_foot,
                                   l_foot,    l_foot,    r_foot, r_foot,
                                   r_foot,    r_foot};
  Matrix3Xd contact_pts_world(contact_pts.rows(), contact_pts.cols());

  int count = 0;                // counts number of loops
  constraint_array.pop_back();  // kc_kuasi_4pts
  constraint_array.pop_back();  // kc_rfoot_pos
  constraint_array.pop_back();  // kc_lfoot_pos
  double lambda_l = 0;
  double lambda_r = 0;
  int nTi = kN_per_step;
  Eigen::MatrixXd q_nom_i(tree->get_num_positions(), nTi);
  MatrixXd q_sol_i(tree->get_num_positions(), nTi);
  q_sol_i = q_sol1.rightCols(nTi);
  q_nom_i = squat_pose_2.replicate(1, nTi);
  std::vector<MatrixXd> q_sols;

  while (count < 8) {
    VectorXd q_cur = q_sol_i.rightCols(1);
    KinematicsCache<double> cache = tree->doKinematics(q_cur);
    Vector3d center_of_mass = tree->centerOfMass(cache);
    TransformPtsBatch(tree.get(), cache, contact_pts, bodies,
                      &contact_pts_world);
    if (IsInsideFeet(contact_pts_world, center_of_mass)) {
      cout << "CG is inside feet polygon." << endl;
      break;
    } else {
      cout << "CG is outside feet polygon." << endl;
    }

    /* constraints always active:
     * - arm posture constraint
     * - arm position constraint
     *
     * constraints depending on left/right
     * - 3 pts quasistatic
     * - left(right) foot fixed
     * - right(left) foot moving "forward"
     */

    std::vector<double> ti;
    for (int i = 0; i < nTi; i++) ti.push_back(1.0 / kN_per_step * (i + 1));
    Vector2d tspani(0, 1);
    Vector2d tspani1(0, 0.5);
    Vector2d tspani2(0.5, 1);
    QuasiStaticConstraint kc_quasi_3pts_i(tree.get(), tspani2);
    QuasiStaticConstraint kc_quasi_4pts_i(tree.get(), tspani1);

    kc_quasi_3pts_i.setShrinkFactor(0.9);
    kc_quasi_3pts_i.setActive(true);
    kc_quasi_4pts_i.setShrinkFactor(0.9);
    kc_quasi_4pts_i.setActive(true);

    kc_quasi_3pts_i.addContact(1, &l_forearm, &leftArmContactPts);
    kc_quasi_3pts_i.addContact(1, &r_forearm, &rightArmContactPts);

    kc_quasi_4pts_i.addContact(1, &l_forearm, &leftArmContactPts);
    kc_quasi_4pts_i.addContact(1, &r_forearm, &rightArmContactPts);
    kc_quasi_4pts_i.addContact(1, &r_foot, &r_foot_pts);
    kc_quasi_4pts_i.addContact(1, &l_foot, &l_foot_pts);

    // move right foot forward
    Vector3d rfoot_origin_forward;
    rfoot_origin_forward << convex_combination(right_start.head(2),
                                               right_end.head(2), lambda_r),
        (right_start(2) + right_end(2)) / 2;
    lambda_r += 0.2;
    cout << "rfoot_origin_forward " << endl << rfoot_origin_forward << endl;
    Vector3d rfoot_origin_forward_lb = rfoot_origin_forward;
    rfoot_origin_forward_lb(0) -= 0.05;
    rfoot_origin_forward_lb(1) -= 0.05;
    Vector3d rfoot_origin_forward_ub = rfoot_origin_forward;
    rfoot_origin_forward_ub(0) += 0.05;
    rfoot_origin_forward_ub(1) += 0.05;
    WorldPositionConstraint kc_rfoot_forward(tree.get(), r_foot, origin,
                                             rfoot_origin_forward_lb,
                                             rfoot_origin_forward_ub, tspani2);
    // move left foot forward
    Vector3d lfoot_origin_forward;
    lfoot_origin_forward << convex_combination(left_start.head(2),
                                               left_end.head(2), lambda_l),
        (left_start(2) + left_end(2)) / 2;
    lambda_l += 0.2;
    cout << "lfoot_origin_forward " << endl << lfoot_origin_forward << endl;
    Vector3d lfoot_origin_forward_lb = lfoot_origin_forward;
    lfoot_origin_forward_lb(0) -= 0.05;
    lfoot_origin_forward_lb(1) -= 0.05;
    Vector3d lfoot_origin_forward_ub = lfoot_origin_forward;
    lfoot_origin_forward_ub(0) += 0.05;
    lfoot_origin_forward_ub(1) += 0.05;
    WorldPositionConstraint kc_lfoot_forward(tree.get(), l_foot, origin,
                                             lfoot_origin_forward_lb,
                                             lfoot_origin_forward_ub, tspani2);
    // fix right foot
    Vector3d rfoot_origin_fixed =
        tree->transformPoints(cache, origin, r_foot, 0);
    Vector3d rfoot_origin_fixed_lb = rfoot_origin_fixed;
    Vector3d rfoot_origin_fixed_ub = rfoot_origin_fixed;
    WorldPositionConstraint kc_rfoot_fixed(tree.get(), r_foot, origin,
                                           rfoot_origin_fixed_lb,
                                           rfoot_origin_fixed_ub, tspani);
    // fix left foot
    Vector3d lfoot_origin_fixed =
        tree->transformPoints(cache, origin, l_foot, 0);
    Vector3d lfoot_origin_fixed_lb = lfoot_origin_fixed;
    Vector3d lfoot_origin_fixed_ub = lfoot_origin_fixed;
    WorldPositionConstraint kc_lfoot_fixed(tree.get(), l_foot, origin,
                                           lfoot_origin_fixed_lb,
                                           lfoot_origin_fixed_ub, tspani);

    // do this only for first loop
    if (count == 0) constraint_array.push_back(&kc_quasi_4pts_i);

    // decides whether to move left or right feet
    if (count % 2) {
      // move right foot
      constraint_array.push_back(&kc_rfoot_forward);
      // fix left foot
      constraint_array.push_back(&kc_lfoot_fixed);
      kc_quasi_3pts_i.addContact(1, &l_foot, &l_foot_pts);
    } else {
      // move left foot
      constraint_array.push_back(&kc_lfoot_forward);
      // fix right foot
      constraint_array.push_back(&kc_rfoot_fixed);
      kc_quasi_3pts_i.addContact(1, &r_foot, &r_foot_pts);
    }
    constraint_array.push_back(&kc_quasi_3pts_i);
    std::vector<int> info_i(nTi, 0);
    MatrixXd q_seed_i = q_cur.replicate(1, nTi);

    inverseKinPointwise(tree.get(), nTi, ti.data(), q_seed_i, q_nom_i,
                        constraint_array.size(), constraint_array.data(),
                        ikoptions, &q_sol_i, info_i.data(),
                        &infeasible_constraint);

    for (int i = 0; i < info_i.size(); i++) {
      cout << "info_i" << count << " " << i << ": " << info_i[i] << endl;
    }
    q_sols.push_back(q_sol_i);
    // clean up
    count++;
    constraint_array.pop_back();  // kc_kuqsi_3pts_i
    constraint_array.pop_back();  // kc_rfoot_fixed/forward
    constraint_array.pop_back();  // kc_lfoot_fixed/forward
  }
  //cout << "constraint_array_size = " << constraint_array.size() << endl;

  Eigen::MatrixXd q_sol(tree->get_num_positions(), nT1 + count * nTi);
  q_sol.leftCols(nT1) = q_sol1;
  for (int i = 0; i < count; i++) {
    q_sol.block(0, i * nTi + nT1, tree->get_num_positions(), nTi) = q_sols[i];
  }
  // q_sol.rightCols(nT2) = q_sol2;
  std::vector<double> t(t1);
  for (int i = 0; i < count; i++) {
    for (int j = 0; j < kN_per_step; j++)
      t.push_back(t[nT1-1] + 1.0 / kN_per_step * (kN_per_step * i + (j + 1)));
  }

  // writing results to a file
  std::ofstream myfile;
  myfile.open("results.txt");
  if (myfile.is_open()) {
    for (int i = 0; i < t.size(); i++) {
      myfile << t[i] << " ";
    }
    myfile << endl;
    myfile << q_sol;
    myfile.close();
  } else
    cout << "unable to open file \"results.txt\"" << endl;

  // generate the CoM and support polygon for each q_sol
  myfile.open("contact_pts.txt");
  if (myfile.is_open()) {
    for (int i = 0; i < t.size(); i++) {
      KinematicsCache<double> cache = tree->doKinematics(q_sol.col(i));
      TransformPtsBatch(tree.get(), cache, contact_pts, bodies,
                        &contact_pts_world);
      Vector3d center_of_mass = tree->centerOfMass(cache);
      myfile << t[i] << endl;
      myfile << center_of_mass << endl;
      myfile << FindCenter(contact_pts_world) << endl;
      myfile << contact_pts_world << endl;
      cout << "t= " << t[i] << " is inside feet?\t"
           << IsInsideFeet(contact_pts_world, center_of_mass) << endl;
      cout << " hands position " << endl;
      cout << tree->transformPoints(cache, leftArmContactPts, l_forearm, 0)
           << endl;
      cout << tree->transformPoints(cache, rightArmContactPts, r_forearm, 0)
           << endl;
    }
    myfile.close();
  } else
    cout << "unable to open file \"contact_pts.txt\"" << endl;

  // show it in drake visualizer
  std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory =
      std::make_unique<PiecewisePolynomialTrajectory>(q_sol, t);

  // VectorXd x(tree->get_num_positions() + tree->get_num_velocities());
  VectorXd x(tree->get_num_velocities());
  x.setZero();
  // x.head(q_sol1.size()) = q_sol1;

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  auto input_mux = builder.AddSystem<systems::Multiplexer<double>>(
      std::vector<int>{tree->get_num_positions(), tree->get_num_velocities()});
  auto traj_source =
      builder.AddSystem<systems::TrajectorySource<double>>(*poly_trajectory);
  auto source = builder.AddSystem<systems::ConstantVectorSource>(x);
  builder.Connect(traj_source->get_output_port(0),
                  input_mux->get_input_port(0));
  builder.Connect(source->get_output_port(), input_mux->get_input_port(1));
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);

  builder.Connect(input_mux->get_output_port(0), publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.StepTo(10);

  // auto context = diagram->CreateDefaultContext();
  // auto output = diagram->AllocateOutput(*context);
  // diagram->Publish(*context);

  return 0;
}

}  // namespace
}  // namespace valkyrie
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::valkyrie::DoMain(); }
