// finds a standing posture of Valkyrie by solving IK and displays it in drake
// visualizer.

#include <iostream>
#include <numeric>

#include "gtest/gtest.h"

// Includes for IK solver.
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;

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
  RigidBody* joint_child_body = tree->FindChildBodyOfJoint(name);
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

GTEST_TEST(ValkyrieIK_Test, ValkyrieIK_Test_StandingPose_Test) {
  std::shared_ptr<RigidBodyTreed> tree = std::make_shared<RigidBodyTreed>(
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_sim_drake_one_neck_dof_additional_contact_pts.urdf",
      drake::multibody::joints::kRollPitchYaw);

  for (int i = 0; i < tree->get_num_bodies(); i++)
    std::cout << i << " " << tree->getBodyOrFrameName(i) << std::endl;

  // Setting up constraints, based on testIKMoreConstraints.cpp and
  // director-generated M-file.
  const double inf = std::numeric_limits<double>::infinity();
  Vector2d tspan;
  tspan << 0, inf;

  VectorXd reach_start(tree->get_num_positions());
  reach_start << 0.0,  // base_x
      0.0,             // base_y
      1.025,           // base_z
      0.0,             // base_roll
      0.0,             // base_pitch
      0.0,             // 5 base_yaw
      0.0,             // 6 torsoYaw
      0.0,             // 7 torsoPitch
      0.0,             // 8 torsoRoll
      0.0,             // 9 lowerNeckPitch
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
      0.0,                  // 26 rightHipYaw
      0.0,                  // 27 rightHipRoll
      -0.49,                // 28 rightHipPitch
      1.205,                // 29 rightKneePitch
      -0.71,                // 30 rightAnklePitch
      0.0,                  // 31 rightAnkleRoll
      0.0,                  // 32 leftHipYaw
      0.0,                  // 33 leftHipRoll
      -0.49,                // 34 leftHipPitch
      1.205,                // 35 leftKneePitch
      -0.71,                // 36 leftAnklePitch
      0.0;                  // 37 leftAnkleRoll

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

      -120.0/180*M_PI,  // 12 rightShoulderPitch [-130, -40]
      80.0/180*M_PI,   // 13 rightShoulderRoll  [70, 85]
      0.05,   // 14 rightShoulderYaw [fixed]
      0.0,   // 15 rightElbowPitch [-6, 120]
      0.0,   // 16 rightForearmYaw [fixed]
      0.0,    // 17 rightWristRoll [fixed]
      -0.49,  // 18 rightWristPitch [fixed]

      -120.0/180*M_PI,  // 19 leftShoulderPitch [-130 -40]
      -80.0/180*M_PI,  // 20 leftShoulderRoll [-85, -70]
      -0.05,  // 21 leftShoulderYaw
      0.0,  // 22 leftElbowPitch [-120, 6]
      0.0,   // 23 leftForearmYaw
      0.0,    // 24 leftWristRoll
      0.49,   // 25 LeftWristPitch

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

  KinematicsCache<double> cache = tree->doKinematics(prone_pose);

  // 1 Neck Posture Constraint, posture constraints are imposed on q
  PostureConstraint kc_posture_neck(tree.get(), tspan);
  std::vector<int> neck_idx;
  FindJointAndInsert(tree.get(), "lowerNeckPitch", &neck_idx);
  FindJointAndInsert(tree.get(), "neckYaw", &neck_idx);
  FindJointAndInsert(tree.get(), "upperNeckPitch", &neck_idx);
  VectorXd neck_lb = VectorXd::Zero(neck_idx.size());
  VectorXd neck_ub = VectorXd::Zero(neck_idx.size());
  kc_posture_neck.setJointLimits(neck_idx.size(), neck_idx.data(), neck_lb,
                                 neck_ub);

  // 2 Left foot position and orientation constraint, position and orientation
  // constraints are imposed on frames/bodies
  const Vector3d origin(0, 0, 0);

  int l_foot = tree->FindBodyIndex("leftFoot");
  Vector4d lfoot_quat(1, 0, 0, 0);
  auto lfoot_pos0 = tree->transformPoints(cache, origin, l_foot, 0);
  Vector3d lfoot_pos_lb(-inf, -inf, lfoot_pos0(2));
  // Position and quaternion constraints are relaxed to make the problem
  // solvable by IPOPT.
  lfoot_pos_lb -= 0.0001 * Vector3d::Ones();
  std::cout << "lfoot_pos_lb: " << std::endl << lfoot_pos_lb << std::endl;
  Vector3d lfoot_pos_ub(inf, inf, lfoot_pos0(2));
  lfoot_pos_ub += 0.0001 * Vector3d::Ones();
  std::cout << "lfoot_pos_ub: " << std::endl << lfoot_pos_ub << std::endl;
  WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, origin, lfoot_pos_lb,
                                       lfoot_pos_ub, tspan);
  double tol = 0.5 / 180 * M_PI;
  WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol, tspan);

  // 3 Right foot position and orientation constraint
  int r_foot = tree->FindBodyIndex("rightFoot");
  auto rfoot_pos0 = tree->transformPoints(cache, origin, r_foot, 0);
  Vector4d rfoot_quat(1, 0, 0, 0);
  Vector3d rfoot_pos_lb(-inf, -inf, rfoot_pos0(2));
  rfoot_pos_lb -= 0.0001 * Vector3d::Ones();
  Vector3d rfoot_pos_ub(inf, inf, rfoot_pos0(2));
  rfoot_pos_ub += 0.0001 * Vector3d::Ones();

  WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, origin, rfoot_pos_lb,
                                       rfoot_pos_ub, tspan);
  WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol, tspan);


  // Pelvis height constraint
  int pelvis = tree->FindBodyIndex("Pelvis");
  Vector3d pelvis_pos_lb(-inf, -inf, 0);
  Vector3d pelvis_pos_ub(inf, inf, 0.5);
  WorldPositionConstraint kc_pelvis_pos(tree.get(), pelvis, origin,
                                       pelvis_pos_lb, pelvis_pos_ub, tspan);


  // 4 Torso posture constraint
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

  // 5 knee posture constraint
  PostureConstraint kc_posture_knee(tree.get(), tspan);
  std::vector<int> knee_idx;
  FindJointAndInsert(tree.get(), "leftKneePitch", &knee_idx);
  FindJointAndInsert(tree.get(), "rightKneePitch", &knee_idx);
  Vector2d knee_lb(-4.0 / 180 * M_PI, -4.0 / 180 * M_PI);
  Vector2d knee_ub(115.0 / 180 * M_PI, 115.0 / 180 * M_PI);
  kc_posture_knee.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

  // 6 Left arm posture constraint

  PostureConstraint kc_posture_larm(tree.get(), tspan);
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
  larm_ub[0] = -40.0/180*M_PI;
  larm_ub[1] = -70.0/180*M_PI;
  larm_ub[3] = 6.0/180*M_PI;

  larm_lb[0] = -130.0/180*M_PI;
  larm_lb[1] = -85.0/180*M_PI;
  larm_lb[3] = -120.0/180*M_PI;

  kc_posture_larm.setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);


  // 7 Right arm posture constraint
  PostureConstraint kc_posture_rarm(tree.get(), tspan);
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
  larm_ub[0] = -40.0/180*M_PI;
  larm_ub[1] = 85.0/180*M_PI;
  larm_ub[3] = 120.0/180*M_PI;

  larm_lb[0] = -130.0/180*M_PI;
  larm_lb[1] = 70.0/180*M_PI;
  larm_lb[3] = -6.0/180*M_PI;

  kc_posture_rarm.setJointLimits(7, rarm_idx.data(), rarm_lb, rarm_ub);


  // 8 Quasistatic constraint
  QuasiStaticConstraint kc_quasi(tree.get(), tspan);
  kc_quasi.setShrinkFactor(0.9);
  kc_quasi.setActive(true);

  auto leftFootPtr = tree->FindBody("leftFoot");
  Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
  Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
  kc_quasi.addContact(1, &l_foot, &l_foot_pts);

  auto rightFootPtr = tree->FindBody("rightFoot");
  Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
  Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
  kc_quasi.addContact(1, &r_foot, &r_foot_pts);

  auto leftArmPtr = tree->FindBody("leftForearmLink");
  Matrix3Xd leftArmContactPts = leftArmPtr->get_contact_points();
  std::cout << "left arm contact pts: " << std::endl
            << leftArmContactPts << std::endl;
  int l_forearm = tree->FindBodyIndex("leftForearmLink");
  kc_quasi.addContact(1, &l_forearm, &leftArmContactPts);

  auto rightArmPtr = tree->FindBody("rightForearmLink");
  Matrix3Xd rightArmContactPts = rightArmPtr->get_contact_points();
  std::cout << "right arm contact pts: " << std::endl
            << rightArmContactPts << std::endl;
  int r_forearm = tree->FindBodyIndex("rightForearmLink");
  kc_quasi.addContact(1, &r_forearm, &rightArmContactPts);

  // 9 leftForeArm position constraint
  Vector3d lforearm_pos_lb(-inf, -inf, 0);
  Vector3d lforearm_pos_ub(inf, inf, 0);
  WorldPositionConstraint kc_lforearm_pos(tree.get(), l_forearm,
                                          leftArmContactPts, lforearm_pos_lb,
                                          lforearm_pos_ub, tspan);

  // 9 leftForeArm position constraint
  Vector3d rforearm_pos_lb(-inf, -inf, 0);
  Vector3d rforearm_pos_ub(inf, inf, 0);
  WorldPositionConstraint kc_rforearm_pos(tree.get(), r_forearm,
                                          rightArmContactPts, rforearm_pos_lb,
                                          rforearm_pos_ub, tspan);

  // 11 no collision constraint
  std::vector<int> active_body_idx;
  // active_body_idx.push_back(l_foot);
  // active_body_idx.push_back(r_foot);
  std::set<std::string> active_group_names;
  active_group_names.insert("l_leg");
  active_group_names.insert("r_leg");
  active_group_names.insert("l_arm");
  AllBodiesClosestDistanceConstraint no_collision_feet(
      tree.get(), 0.0, inf, active_body_idx, active_group_names, tspan);
  // -----------------solve-----------------------------------------------------
  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&kc_posture_neck);
  constraint_array.push_back(&kc_lfoot_pos);
  constraint_array.push_back(&kc_lfoot_quat);
  constraint_array.push_back(&kc_rfoot_pos);
  constraint_array.push_back(&kc_rfoot_quat);
  constraint_array.push_back(&kc_posture_torso);
  constraint_array.push_back(&kc_posture_knee);
  constraint_array.push_back(&kc_posture_larm);
  constraint_array.push_back(&kc_posture_rarm);
  constraint_array.push_back(&kc_quasi);
  constraint_array.push_back(&kc_lforearm_pos);
  constraint_array.push_back(&kc_rforearm_pos);
  constraint_array.push_back(&no_collision_feet);
  //constraint_array.push_back(&kc_pelvis_pos);

  // setting Q

  std::cout << "postions and names: " << std::endl;
  for (int i = 0; i < tree->get_num_positions(); i++)
    std::cout << i << " " << tree->get_position_name(i) << std::endl;


  VectorXd cost(tree->get_num_positions());
  cost.setOnes();
  /*
  const int left_hip_pitch_idx = 32;
  const int right_hip_pitch_idx = 26;
  const int left_knee_pitch_idx = 33;
  const int right_knee_pitch_idx = 27;
  const int left_ankle_pitch_idx = 34;
  const int right_ankle_pitch_idx = 28;
  const int torso_roll_idx = 8;
  const int torso_pitch_idx = 7;
  const int torso_yaw_idx = 6;

  std::vector<int> lower_body_pitch_idx;
  lower_body_pitch_idx.push_back(left_hip_pitch_idx);
  lower_body_pitch_idx.push_back(right_hip_pitch_idx);
  lower_body_pitch_idx.push_back(left_knee_pitch_idx);
  lower_body_pitch_idx.push_back(right_knee_pitch_idx);
  lower_body_pitch_idx.push_back(left_ankle_pitch_idx);
  lower_body_pitch_idx.push_back(right_ankle_pitch_idx);

  std::vector<int> torso_rotation_idx = {torso_yaw_idx, torso_roll_idx,
                                         torso_pitch_idx};

  std::vector<int> base_idx = {0, 1, 2, 3, 4, 5};
  for (const int& i : base_idx) cost(i) = 0;
  for (const int& i : torso_rotation_idx) cost(i) = 0.1;
  for (const int& i : lower_body_pitch_idx) cost(i) = 0;
  */

  // display all costs
  std::cout << "costs are:" << std::endl;
  for (int i = 0; i < cost.size(); i++) {
    std::cout << cost(i) << " " << tree->get_position_name(i) << i << std::endl;
  }

  Eigen::MatrixXd Q = cost.asDiagonal();
  Eigen::MatrixXd Qv = 0.05 * Q;
  Eigen::MatrixXd Qa = 0.05 * Q;


  IKoptions ikoptions(tree.get());
  ikoptions.setQ(Q);
  ikoptions.setQa(Qa);
  ikoptions.setQv(Qv);

  VectorXd q_sol(tree->get_num_positions());
  VectorXd q_nom = prone_pose;
  int info;
  std::vector<std::string> infeasible_constraint;
  inverseKin(tree.get(), q_nom, q_nom, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_sol, &info,
             &infeasible_constraint);

  // After solving
  Vector3d com = tree->centerOfMass(cache);
  EXPECT_EQ(info, 1);
  EXPECT_GT(com(2), 0);

  // show it in drake visualizer
  VectorXd x(tree->get_num_positions() + tree->get_num_velocities());
  x.setZero();
  x.head(q_sol.size()) = q_sol;

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::ConstantVectorSource>(x);
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(source->get_output_port(), publisher->get_input_port(0));
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);
  diagram->Publish(*context);
}

}  // namespace
}  // namespace valkyrie
}  // namespace examples
}  // namespace drake
