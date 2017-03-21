// finds a standing posture of Valkyrie by solving IK and displays it in drake
// visualizer.

#include <iostream>
#include <memory>
#include <numeric>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

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
  RigidBody<double>* joint_child_body = tree->FindChildBodyOfJoint(name);
  int num_positions = joint_child_body->getJoint().get_num_positions();
  std::vector<int> ret(static_cast<size_t>(num_positions));

  // Since the joint position states are located in a contiguous region of the
  // the rigid body tree's state vector, fill the return vector with
  // sequentially increasing indices starting at
  // `joint_child_body->get_position_start_index()`.
  std::iota(ret.begin(), ret.end(), joint_child_body->get_position_start_index
  ());
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
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      multibody::joints::kRollPitchYaw, tree.get());

  // Setting up constraints, based on testIKMoreConstraints.cpp and
  // director-generated M-file.
  double inf = std::numeric_limits<double>::infinity();
  Vector2d tspan;
  tspan << 0, inf;

  VectorXd reach_start(tree->get_num_positions());
  // There are two versions of Valkyrie models. The one in
  // Drake has two DOFs less (the two commented out) than the other. The
  // indices in the comment are for reference only and are not hard coded.
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

  KinematicsCache<double> cache = tree->doKinematics(reach_start);

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
  Vector3d lfoot_pos_lb = lfoot_pos0;
  // Position and quaternion constraints are relaxed to make the problem
  // solvable by IPOPT.
  lfoot_pos_lb -= 0.0001*Vector3d::Ones();
  Vector3d lfoot_pos_ub = lfoot_pos0;
  lfoot_pos_ub += 0.0001*Vector3d::Ones();
  WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, origin, lfoot_pos_lb,
                                       lfoot_pos_ub, tspan);
  double tol = 0.5 / 180 * M_PI;
  WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol, tspan);

  // 3 Right foot position and orientation constraint
  int r_foot = tree->FindBodyIndex("rightFoot");
  auto rfoot_pos0 = tree->transformPoints(cache, origin, r_foot, 0);
  Vector4d rfoot_quat(1, 0, 0, 0);
  Vector3d rfoot_pos_lb = rfoot_pos0;
  rfoot_pos_lb -= 0.0001*Vector3d::Ones();
  Vector3d rfoot_pos_ub = rfoot_pos0;
  rfoot_pos_ub += 0.0001*Vector3d::Ones();

  WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, origin, rfoot_pos_lb,
                                       rfoot_pos_ub, tspan);
  WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol, tspan);

  // 4 Torso posture constraint
  PostureConstraint kc_posture_torso(tree.get(), tspan);
  std::vector<int> torso_idx;
  FindJointAndInsert(tree.get(), "torsoYaw", &torso_idx);
  FindJointAndInsert(tree.get(), "torsoPitch", &torso_idx);
  FindJointAndInsert(tree.get(), "torsoRoll", &torso_idx);
  Vector3d torso_nominal = Vector3d::Zero();
  Vector3d torso_half_range(15.0 / 180 * M_PI, 25.0 / 180 * M_PI, inf);
  Vector3d torso_lb = torso_nominal - torso_half_range;
  Vector3d torso_ub = torso_nominal + torso_half_range;
  torso_lb(1) = -5.0 / 180 * M_PI;
  kc_posture_torso.setJointLimits(3, torso_idx.data(), torso_lb, torso_ub);

  // 5 knee posture constraint
  PostureConstraint kc_posture_knee(tree.get(), tspan);
  std::vector<int> knee_idx;
  FindJointAndInsert(tree.get(), "leftKneePitch", &knee_idx);
  FindJointAndInsert(tree.get(), "rightKneePitch", &knee_idx);
  Vector2d knee_lb(-4.0/180*M_PI, -4.0/180*M_PI);
  Vector2d knee_ub(115.0/180*M_PI, 115.0/180*M_PI);
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
  Eigen::Matrix<double, 7, 1> larm_lb;
  larm_lb.setZero();
  for (int i = 0; i < 7; i++) larm_lb(i) = reach_start(larm_idx[i]);
  Eigen::Matrix<double, 7, 1> larm_ub = larm_lb;
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
  Eigen::Matrix<double, 7, 1> rarm_lb;
  rarm_lb.setZero();
  for (int i = 0; i < 7; i++) rarm_lb(i) = reach_start(rarm_idx[i]);
  Eigen::Matrix<double, 7, 1> rarm_ub = rarm_lb;
  kc_posture_rarm.setJointLimits(7, rarm_idx.data(), rarm_lb, rarm_ub);

  // 8 Quasistatic constraint
  QuasiStaticConstraint kc_quasi(tree.get(), tspan);
  kc_quasi.setShrinkFactor(0.4);
  kc_quasi.setActive(true);

  auto leftFootPtr = tree->FindBody("leftFoot");
  Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
  Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
  kc_quasi.addContact(1, &l_foot, &l_foot_pts);

  auto rightFootPtr = tree->FindBody("rightFoot");
  Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
  Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
  kc_quasi.addContact(1, &r_foot, &r_foot_pts);

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

  IKoptions ikoptions(tree.get());
  VectorXd q_sol(tree->get_num_positions());
  VectorXd q_nom = reach_start;
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
