// finds a standing posture of Valkyrie by solving IK and displays it in drake
// visualizer

#include <iostream>

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

// Includes for IK solver
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix3Xd;

namespace drake {
namespace examples {
namespace Valkyrie {
namespace {

/* Finds and returns the indices within the state vector of @p tree that contain
 * the position states of a joint named @p name. The model instance ID is
 * ignored in this search (joints belonging to all model instances are
 * searched).
 */
std::vector<int> GetJointPositionVectorIndices(const RigidBodyTree* tree,
                                               const std::string& name) {
  RigidBody* joint_child_body = tree->FindChildBodyOfJoint(name);
  int num_positions = joint_child_body->getJoint().get_num_positions();
  std::vector<int> ret(static_cast<size_t>(num_positions));

  // Since the joint position states are located in a contiguous region of the
  // the rigid body tree's state vector, fill the return vector with
  // sequentially increasing indices starting at
  // `joint_child_body->get_position_start_index()`.
  iota(ret.begin(), ret.end(), joint_child_body->get_position_start_index());
  return ret;
}

void findJointAndInsert(const RigidBodyTree* model, const std::string& name,
                        std::vector<int> *const position_list) {
  auto position_indices = GetJointPositionVectorIndices(model, name);

  position_list->insert(position_list->end(), position_indices.begin(),
                       position_indices.end());
}

int do_main() {
  std::shared_ptr<RigidBodyTree> tree = std::make_shared<RigidBodyTree>(
      drake::GetDrakePath() +
          "/examples/Valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf",
      systems::plants::joints::kRollPitchYaw);
  // tests
  /*
  std::cout << "Number of positions: "<< tree2->get_num_positions() <<
  std::endl;
  for(int i=0;i<tree2->get_num_positions();i++) {
      std::cout << i << ":" << tree2->get_position_name(i) << std::endl;
  }

  std::cout << "Number of velocities: "<< tree2->get_num_velocities() <<
  std::endl;
  for(int i=0;i<tree2->get_num_velocities();i++) {
      std::cout << i << ":" << tree2->get_velocity_name(i) << " " <<
  tree2->get_position_name(i) << std::endl;
  }

  std::cout << "Number of bodies: " << tree2->get_num_bodies() << std::endl;
  for(int i=0;i<tree2->get_num_bodies();i++) {
      std::cout << i << ":" << tree2->getBodyOrFrameName(i) << std::endl;
  }

  auto leftPalmPtr = tree2->FindBody("leftPalm");
  Eigen::Matrix3Xd leftPalmContactPts = leftPalmPtr->get_contact_points();
  std::cout << "LeftPalmContactPts:" << std::endl;
  std::cout << leftPalmContactPts << std::endl;
  */

  // setting up constraints, based on testIKMoreConstraints.cpp and
  // director-generated M-file.
  double inf = std::numeric_limits<double>::infinity();
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

  KinematicsCache<double> cache = tree->doKinematics(reach_start);

  // 1 Neck Posture Constraint, posture constraints are imposed on q
  PostureConstraint kc_posture_neck(tree.get(), tspan);
  std::vector<int> neck_idx;
  findJointAndInsert(tree.get(), "lowerNeckPitch", &neck_idx);
  findJointAndInsert(tree.get(), "neckYaw", &neck_idx);
  findJointAndInsert(tree.get(), "upperNeckPitch", &neck_idx);
  VectorXd neck_lb = VectorXd::Zero(neck_idx.size());
  VectorXd neck_ub = VectorXd::Zero(neck_idx.size());
  kc_posture_neck.setJointLimits(neck_idx.size(), neck_idx.data(), neck_lb,
                                 neck_ub);

  // 2 left foot position and orientation constraint, position and orientation
  // constraints are imposed on frames/bodies
  int l_foot = tree->FindBodyIndex("leftFoot");
  // Vector4d lfoot_quat = tree->relativeQuaternion(cache, l_foot, 0);
  Vector4d lfoot_quat(1, 0, 0, 0);
  // std::cout << "Relative quaternion between left foot and world" << std::endl
  // << lfoot_quat << std::endl;

  const Vector3d origin(0, 0, 0);
  // std::cout << "origin is" << std::endl << origin << std::endl;
  auto lfoot_pos0 = tree->transformPoints(cache, origin, l_foot, 0);
  // std::cout << "Relative position between left foot and world" << std::endl
  // << lfoot_pos0 << std::endl;
  Vector3d lfoot_pos_lb = lfoot_pos0;
  lfoot_pos_lb(0) -= 0.0001;
  lfoot_pos_lb(1) -= 0.0001;
  lfoot_pos_lb(2) -= 0.0001;
  Vector3d lfoot_pos_ub = lfoot_pos0;
  lfoot_pos_ub(0) += 0.0001;
  lfoot_pos_ub(1) += 0.0001;
  lfoot_pos_ub(2) += 0.0001;
  WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, origin, lfoot_pos_lb,
                                       lfoot_pos_ub, tspan);
  double tol = 0.0017453292519943296;
  WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol, tspan);

  // 3 right foot position and orientation constraint
  int r_foot = tree->FindBodyIndex("rightFoot");
  // Vector4d rfoot_quat = tree->relativeQuaternion(cache, r_foot, 0);
  Vector4d rfoot_quat(1, 0, 0, 0);
  // std::cout << "Relative quaternion between right foot and world" <<
  // std::endl << rfoot_quat << std::endl;

  auto rfoot_pos0 = tree->transformPoints(cache, origin, r_foot, 0);
  // std::cout << "Relative position between right foot and world" << std::endl
  // << rfoot_pos0 << std::endl;
  Vector3d rfoot_pos_lb = rfoot_pos0;
  rfoot_pos_lb(0) -= 0.0001;
  rfoot_pos_lb(1) -= 0.0001;
  rfoot_pos_lb(2) -= 0.0001;
  Vector3d rfoot_pos_ub = rfoot_pos0;
  rfoot_pos_ub(0) += 0.0001;
  rfoot_pos_ub(1) += 0.0001;
  rfoot_pos_ub(2) += 0.0001;
  WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, origin, rfoot_pos_lb,
                                       rfoot_pos_ub, tspan);
  WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol, tspan);

  // 4 torso posture constraint
  PostureConstraint kc_posture_torso(tree.get(), tspan);
  std::vector<int> torso_idx;
  findJointAndInsert(tree.get(), "torsoYaw", &torso_idx);
  findJointAndInsert(tree.get(), "torsoPitch", &torso_idx);
  findJointAndInsert(tree.get(), "torsoRoll", &torso_idx);
  // std::cout << "torso indices " << torso_idx[0] << torso_idx[1] <<
  // torso_idx[2] << std::endl;
  Vector3d torso_nominal = Vector3d::Zero(3);
  Vector3d torso_half_range(0.2617993877991494, 0.4363323129985824, inf);
  Vector3d torso_lb = torso_nominal - torso_half_range;
  Vector3d torso_ub = torso_nominal + torso_half_range;
  torso_lb(1) = -0.08726646259971647;
  kc_posture_torso.setJointLimits(3, torso_idx.data(), torso_lb, torso_ub);
  // std::cout << "inf+1" << inf + 1.0 << std::endl;
  // std::cout << "-inf+1" << -inf + 1.0 << std::endl;

  // 5 knee posture constraint
  PostureConstraint kc_posture_knee(tree.get(), tspan);
  std::vector<int> knee_idx;
  findJointAndInsert(tree.get(), "leftKneePitch", &knee_idx);
  findJointAndInsert(tree.get(), "rightKneePitch", &knee_idx);
  Vector2d knee_nominal(reach_start(knee_idx[0]), reach_start(knee_idx[1]));
  Vector2d knee_lb = knee_nominal;
  // knee_lb(0) += 0.60;
  // knee_lb(0) += 0.60;
  Vector2d knee_ub = knee_nominal;
  knee_ub(0) += 1.90;
  knee_ub(1) += 1.90;
  kc_posture_knee.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);
  // std::cout << "knee_idx " << knee_idx[0] << knee_idx[1] << std::endl;
  // std::cout << "knee_nominal_posture" << knee_nominal << std::endl;

  // 6 left arm posture constraint
  PostureConstraint kc_posture_larm(tree.get(), tspan);
  std::vector<int> larm_idx;
  findJointAndInsert(tree.get(), "leftShoulderPitch", &larm_idx);
  findJointAndInsert(tree.get(), "leftShoulderRoll", &larm_idx);
  findJointAndInsert(tree.get(), "leftShoulderYaw", &larm_idx);
  findJointAndInsert(tree.get(), "leftElbowPitch", &larm_idx);
  findJointAndInsert(tree.get(), "leftForearmYaw", &larm_idx);
  findJointAndInsert(tree.get(), "leftWristRoll", &larm_idx);
  findJointAndInsert(tree.get(), "leftWristPitch", &larm_idx);
  // std::cout << "Number of elements in larm_idx " << larm_idx.size() <<
  // std::endl;
  VectorXd larm_lb = VectorXd::Zero(7);
  for (int i = 0; i < 7; i++) larm_lb(i) = reach_start(larm_idx[i]);
  VectorXd larm_ub = larm_lb;
  kc_posture_larm.setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);

  // 7 right arm posture constraint
  PostureConstraint kc_posture_rarm(tree.get(), tspan);
  std::vector<int> rarm_idx;
  findJointAndInsert(tree.get(), "rightShoulderPitch", &rarm_idx);
  findJointAndInsert(tree.get(), "rightShoulderRoll", &rarm_idx);
  findJointAndInsert(tree.get(), "rightShoulderYaw", &rarm_idx);
  findJointAndInsert(tree.get(), "rightElbowPitch", &rarm_idx);
  findJointAndInsert(tree.get(), "rightForearmYaw", &rarm_idx);
  findJointAndInsert(tree.get(), "rightWristRoll", &rarm_idx);
  findJointAndInsert(tree.get(), "rightWristPitch", &rarm_idx);
  // std::cout << "Number of elements in rarm_idx " << rarm_idx.size() <<
  // std::endl;
  VectorXd rarm_lb = VectorXd::Zero(7);
  for (int i = 0; i < 7; i++) rarm_lb(i) = reach_start(rarm_idx[i]);
  VectorXd rarm_ub = rarm_lb;
  kc_posture_rarm.setJointLimits(7, rarm_idx.data(), rarm_lb, rarm_ub);

  // 8 quasistatic constraint
  QuasiStaticConstraint kc_quasi(tree.get(), tspan);
  kc_quasi.setShrinkFactor(0.2);
  kc_quasi.setActive(true);

  auto leftFootPtr = tree->FindBody("leftFoot");
  Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
  Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
  kc_quasi.addContact(1, &l_foot, &l_foot_pts);

  auto rightFootPtr = tree->FindBody("rightFoot");
  Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
  Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
  kc_quasi.addContact(1, &r_foot, &r_foot_pts);

  std::cout << "LeftFootContactPts:" << std::endl;
  std::cout << leftFootContactPts << std::endl;
  std::cout << "lfoot contact pts used for quasistatic constraint" << std::endl;
  std::cout << l_foot_pts << std::endl;

  std::cout << "RightFootContactPts: " << std::endl;
  std::cout << rightFootContactPts << std::endl;
  std::cout << "rfoot contact pts used for quasistatic constraint" << std::endl;
  std::cout << r_foot_pts << std::endl;

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
  VectorXd q_nom = VectorXd::Zero(tree->get_num_positions());
  int info;
  std::vector<std::string> infeasible_constraint;
  inverseKin(tree.get(), q_nom, q_nom, constraint_array.size(),
             constraint_array.data(), ikoptions, &q_sol, &info,
             &infeasible_constraint);
  printf("info = %d\n", info);

  /////////////////////////////////////////
  std::cout
      << "=======================after solving============================"
      << std::endl;
  cache = tree->doKinematics(q_sol);
  Vector3d com = tree->centerOfMass(cache);
  printf("%5.6f\n%5.6f\n%5.6f\n", com(0), com(1), com(2));

  // cout for debugging
  /*
  for(auto & a:infeasible_constraint)
      std::cout << a << std::endl;
  std::cout << std::endl;

  lfoot_quat = tree->relativeQuaternion(cache, l_foot, 0);
  std::cout << "Relative quaternion between left foot and world" << std::endl <<
  lfoot_quat << std::endl;
  rfoot_quat = tree->relativeQuaternion(cache, r_foot, 0);

  std::cout << "Final States: " << std::endl;
  for(int i=0;i<tree->get_num_positions();i++) {
      std::cout << i << ":" << tree->get_position_name(i) << " " << q_sol(i) <<
  " " << reach_start(i) << std::endl;
  }
  */

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
  //  diagram->EvalOutput(*context, output.get());
  diagram->Publish(*context);
  // systems::Simulator<double> simulator(*diagram);
  // simulator.Initialize();
  // simulator.StepTo(1);

  return 0;
}

}  // namespace
}  // namespace Valkyrie
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::Valkyrie::do_main(); }
