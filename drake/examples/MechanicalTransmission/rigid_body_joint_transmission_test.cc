#include <fstream>
#include <iostream>
#include "drake/systems/plants/RigidBodySystem.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/trajectory_logger.h"

using drake::RigidBodySystem;
using Eigen::VectorXd;
using drake::CompareMatrices;
using drake::MatrixCompareType;
using drake::parsers::ModelInstanceIdTable;

namespace drake {
namespace examples {
namespace mechanical_transmission {
namespace {

// Loads mechanical_transmission.urdf, which says that
// joint1_value = 0.5 * joint2_value + 1.0,
// and returns the RigidBodySystem.
std::shared_ptr<RigidBodySystem> ParseMechanicalTransmission(
    ModelInstanceIdTable& model_instance_id_table) {
  auto rigid_body_system = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  model_instance_id_table = rigid_body_system->AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/examples/MechanicalTransmission/mechanical_transmission.urdf",
      DrakeJoint::FIXED);

  rigid_body_system->penetration_stiffness = 3000.0;
  rigid_body_system->penetration_damping = 0;
  return rigid_body_system;
}

// Evaluates the mechanical joint transmission expression
// joint1_value - 0.5 * joint2_value - 1.0, the
// Jacobian J of the constraint, and the term Jdot * v.
void EvaluateMechanicalTransmissionConstraint(
    const std::shared_ptr<RigidBodyTree>& tree, double joint1_pos_val,
    double joint2_pos_val, double joint1_vel_val, double joint2_vel_val,
    Eigen::VectorXd& position_constraint,
    Eigen::MatrixXd& position_constraint_jacobian,
    Eigen::VectorXd& position_constraint_jac_dot_times_v, bool in_terms_of_qdot,
    int model_instance_id = -1) {
  int nq = tree->number_of_positions();
  int nv = tree->number_of_velocities();
  auto link1 = tree->FindChildBodyOfJoint("joint1", model_instance_id);
  auto link2 = tree->FindChildBodyOfJoint("joint2", model_instance_id);
  int joint1_pos_idx = link1->get_position_start_index();
  int joint2_pos_idx = link2->get_position_start_index();
  int joint1_vel_idx = link1->get_velocity_start_index();
  int joint2_vel_idx = link2->get_velocity_start_index();
  VectorXd q0(nq);
  q0.setZero();
  q0(joint1_pos_idx) = joint1_pos_val;
  q0(joint2_pos_idx) = joint2_pos_val;
  VectorXd v0(nv);
  v0.setZero();
  v0(joint1_vel_idx) = joint1_vel_val;
  v0(joint2_vel_idx) = joint2_vel_val;
  auto kinematics_cache = tree->doKinematics(q0, v0, true);
  position_constraint = tree->positionConstraints(kinematics_cache);
  position_constraint_jacobian =
      tree->positionConstraintsJacobian(kinematics_cache, in_terms_of_qdot);
  position_constraint_jac_dot_times_v =
      tree->positionConstraintsJacDotTimesV(kinematics_cache);
  return;
}

// Tests if the mechanical joint transmission in the URDF is parsed correctly by
// the URDF parser.
GTEST_TEST(MechanicalTransmissionTest, ParseMechanicalTransmission) {
  ModelInstanceIdTable model_instance_id_table;
  auto rigid_body_system = ParseMechanicalTransmission(model_instance_id_table);
  auto tree = rigid_body_system->getRigidBodyTree();

  // Joints values [1.0, 0.0] should satisfy the constraint; check if the offset
  // is set correctly.
  Eigen::VectorXd pos_cnstr;
  Eigen::MatrixXd pos_jac;
  Eigen::VectorXd pos_jac_dot_times_v;
  bool in_terms_of_qdot = true;
  EvaluateMechanicalTransmissionConstraint(tree, 1.0, 0.0, 1.0, 0.5, pos_cnstr,
                                           pos_jac, pos_jac_dot_times_v,
                                           in_terms_of_qdot);
  EXPECT_TRUE(CompareMatrices(pos_cnstr, Eigen::Matrix<double, 1, 1>(0.0),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  Eigen::Matrix<double, 1, 2> pos_jac_check;
  pos_jac_check(0, 0) = 1.0;
  pos_jac_check(0, 1) = -0.5;
  EXPECT_TRUE(CompareMatrices(pos_jac, pos_jac_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      pos_jac_dot_times_v, Eigen::Matrix<double, 1, 1>(0.0),
      Eigen::NumTraits<double>::epsilon(), MatrixCompareType::absolute));

  // Joints values [2.0, 2.0] should satisfy the constraint.
  EvaluateMechanicalTransmissionConstraint(tree, 2.0, 2.0, 1.0, 0.5, pos_cnstr,
                                           pos_jac, pos_jac_dot_times_v,
                                           in_terms_of_qdot);
  EXPECT_TRUE(CompareMatrices(pos_cnstr, Eigen::Matrix<double, 1, 1>(0.0),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(pos_jac, pos_jac_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      pos_jac_dot_times_v, Eigen::Matrix<double, 1, 1>(0.0),
      Eigen::NumTraits<double>::epsilon(), MatrixCompareType::absolute));

  // Joints values [2.0, 3.0] should not satisfy the constraint.
  EvaluateMechanicalTransmissionConstraint(tree, 2.0, 3.0, 1.0, 0.5, pos_cnstr,
                                           pos_jac, pos_jac_dot_times_v,
                                           in_terms_of_qdot);
  // The pos_cnstr should be equal to -0.5. We check not only the constraint is
  // violated (not equal to 0), but also in what way the constraint is violated.
  EXPECT_TRUE(CompareMatrices(pos_cnstr, Eigen::Matrix<double, 1, 1>(-0.5),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(pos_jac, pos_jac_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      pos_jac_dot_times_v, Eigen::Matrix<double, 1, 1>(0.0),
      Eigen::NumTraits<double>::epsilon(), MatrixCompareType::absolute));
}

// Tests the simulation with the mechanical transmission.
// From the  <mimic> element in the URDF, joint 1 mimics joint 2 with ratio of
// 0.5 and an offset of 1. For more details, see:
// http://wiki.ros.org/urdf/XML/joint.
//
// The transmission constraint derived from the URDF <mimic> element
// enforces the following:
//
//    joint1_position = ratio * joint2_position + offset
//
// Solving the above equation for ratio, we get:
//
//    ratio = (joint1_position - offset) / joint2_position
//
// Plugging in the values for this particular example, we get:
//
//    0.5 = (joint1_position - 1) / joint2_position
//
// The following check verifies that the above equation is true.
GTEST_TEST(MechanicalTransmissionTest, MechanicalTransmissionSimulation) {
  ModelInstanceIdTable model_instance_id_table;
  auto rigid_body_system = ParseMechanicalTransmission(model_instance_id_table);
  auto tree = rigid_body_system->getRigidBodyTree();
  int nq = tree->number_of_positions();
  drake::SimulationOptions options;
  options.realtime_factor = 0;  // As fast as possible.
  options.initial_step_size = 0.02;

  // Prevents an exception from being thrown when the simulation runs slower
  // than real time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;

  // Instantiates a variable that specifies the duration of the simulation.
  // The default value is 5 seconds.
  double duration = 5.0;

  // Starts the simulation.
  const double kStartTime = 0.0;
  VectorXd x0 = VectorXd::Zero(rigid_body_system->getNumStates());
  // Determines the indices within the `RigidBodySystem's` output vector
  // containing the positions and velocities of the two joints in the URDF.
  int joint1_pos_index =
      tree->FindChildBodyOfJoint("joint1")->get_position_start_index();
  int joint2_pos_index =
      tree->FindChildBodyOfJoint("joint2")->get_position_start_index();
  int joint1_vel_index =
      nq + tree->FindChildBodyOfJoint("joint1")->get_velocity_start_index();
  int joint2_vel_index =
      nq + tree->FindChildBodyOfJoint("joint2")->get_velocity_start_index();

  x0(joint1_pos_index) = 2.0;
  x0(joint2_pos_index) = 2.0;
  x0(joint1_vel_index) = 0.1;
  x0(joint2_vel_index) = 0.2;
  auto trajectory_logger = std::make_shared<
      drake::systems::TrajectoryLogger<RigidBodySystem::StateVector>>(
      rigid_body_system->getNumStates());
  auto sys = drake::cascade(rigid_body_system, trajectory_logger);
  drake::simulate(*sys, kStartTime, duration, x0, options);
  auto trajectory = trajectory_logger->getTrajectory();

  for (size_t i = 0; i < trajectory.time.size(); ++i) {
    EXPECT_NEAR((trajectory.value[i](joint1_pos_index) - 1.0) /
                    trajectory.value[i](joint2_pos_index),
                0.5, 5e-2);
  }
  x0(joint1_pos_index) = 2.0;
  x0(joint2_pos_index) = 2.5;
  x0(joint1_vel_index) = 0.1;
  x0(joint2_vel_index) = 0.18;
  trajectory_logger->clearTrajectory();
  drake::simulate(*sys, kStartTime, duration, x0, options);
  trajectory = trajectory_logger->getTrajectory();
  auto final_state = trajectory.value[trajectory.value.size() - 1];
  EXPECT_NEAR(
      (final_state(joint1_pos_index) - 1.0) / final_state(joint2_pos_index),
      0.5, 5e-2);
}

// Tests if the joint transmission constraint is parsed correctly when we have
// two identical model instances in the same rigid body tree.
GTEST_TEST(MechanicalTransmissionTest,
           ParseMechanicalTransmissionMultipleModelInstance) {
  ModelInstanceIdTable model_instance_id_table1;
  auto rigid_body_system =
      ParseMechanicalTransmission(model_instance_id_table1);
  // Adds a second model instance to the rigid body tree
  auto model_instance_id_table2 = rigid_body_system->AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/examples/MechanicalTransmission/mechanical_transmission.urdf",
      DrakeJoint::FIXED);
  auto tree = rigid_body_system->getRigidBodyTree();

  int model_instance_id1 = model_instance_id_table1.begin()->second;
  int model_instance_id2 = model_instance_id_table2.begin()->second;
  int joint1_model0_vel_index =
      tree->FindChildBodyOfJoint("joint1", model_instance_id1)
          ->get_velocity_start_index();
  int joint2_model0_vel_index =
      tree->FindChildBodyOfJoint("joint2", model_instance_id1)
          ->get_velocity_start_index();
  int joint1_model1_vel_index =
      tree->FindChildBodyOfJoint("joint1", model_instance_id2)
          ->get_velocity_start_index();
  int joint2_model1_vel_index =
      tree->FindChildBodyOfJoint("joint2", model_instance_id2)
          ->get_velocity_start_index();
  // Joint values (2.0,2.0,0.0,0.0) should satisfy the first joint transmission
  // constraint
  Eigen::VectorXd pos_cnstr;
  Eigen::MatrixXd pos_jac;
  Eigen::VectorXd pos_jac_dot_times_v;
  bool in_terms_of_qdot = true;
  Eigen::Matrix<double, 2, 4> pos_jac_check;
  pos_jac_check.setZero();
  pos_jac_check(0, joint1_model0_vel_index) = 1.0;
  pos_jac_check(0, joint2_model0_vel_index) = -0.5;
  pos_jac_check(1, joint1_model1_vel_index) = 1.0;
  pos_jac_check(1, joint2_model1_vel_index) = -0.5;
  Eigen::Vector2d pos_jac_dot_times_v_check;
  pos_jac_dot_times_v_check.setZero();
  EvaluateMechanicalTransmissionConstraint(tree, 2.0, 2.0, 1.0, 0.5, pos_cnstr,
                                           pos_jac, pos_jac_dot_times_v,
                                           in_terms_of_qdot, 0);
  Eigen::Vector2d pos_cnstr_check(0.0, -1.0);
  EXPECT_TRUE(CompareMatrices(pos_cnstr, pos_cnstr_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(pos_jac, pos_jac_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(pos_jac_dot_times_v, pos_jac_dot_times_v_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Joint values (0.0,0.0,2.0,0.0) should satisfy the second joint transmission
  // constraint
  EvaluateMechanicalTransmissionConstraint(tree, 2.0, 2.0, 1.0, 0.5, pos_cnstr,
                                           pos_jac, pos_jac_dot_times_v,
                                           in_terms_of_qdot, 1);
  pos_cnstr_check(0) = -1.0;
  pos_cnstr_check(1) = 0.0;
  EXPECT_TRUE(CompareMatrices(pos_cnstr, pos_cnstr_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(pos_jac, pos_jac_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(pos_jac_dot_times_v, pos_jac_dot_times_v_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Tests if the joint transmission constraint is simulated correctly when we
// have two identical model instances in the same rigid body tree.
GTEST_TEST(MechanicalTransmissionTest,
           SimulateMechanicalTransmissionMultipleModelInstance) {
  ModelInstanceIdTable model_instance_id_table1;
  auto rigid_body_system =
      ParseMechanicalTransmission(model_instance_id_table1);
  // add a second model instance to the rigid body system
  auto model_instance_id_table2 = rigid_body_system->AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/examples/MechanicalTransmission/mechanical_transmission.urdf",
      DrakeJoint::FIXED);
  auto tree = rigid_body_system->getRigidBodyTree();

  int nq = tree->number_of_positions();
  drake::SimulationOptions options;
  options.realtime_factor = 0;  // As fast as possible.
  options.initial_step_size = 0.02;

  // Prevents an exception from being thrown when the simulation runs slower
  // than real time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;

  // Instantiates a variable that specifies the duration of the simulation.
  // The default value is 5 seconds.
  double duration = 5.0;

  // Starts the simulation.
  const double kStartTime = 0.0;
  VectorXd x0 = VectorXd::Zero(rigid_body_system->getNumStates());
  // Determines the indices within the `RigidBodySystem's` output vector
  // containing the positions and velocities of the two joints in the URDF.
  int model_instance_id1 = model_instance_id_table1.begin()->second;
  int model_instance_id2 = model_instance_id_table2.begin()->second;
  int joint1_model0_pos_index =
      tree->FindChildBodyOfJoint("joint1", model_instance_id1)
          ->get_position_start_index();
  int joint2_model0_pos_index =
      tree->FindChildBodyOfJoint("joint2", model_instance_id1)
          ->get_position_start_index();
  int joint1_model0_vel_index =
      nq +
      tree->FindChildBodyOfJoint("joint1", model_instance_id1)
          ->get_velocity_start_index();
  int joint2_model0_vel_index =
      nq +
      tree->FindChildBodyOfJoint("joint2", model_instance_id1)
          ->get_velocity_start_index();
  int joint1_model1_pos_index =
      tree->FindChildBodyOfJoint("joint1", model_instance_id2)
          ->get_position_start_index();
  int joint2_model1_pos_index =
      tree->FindChildBodyOfJoint("joint2", model_instance_id2)
          ->get_position_start_index();
  int joint1_model1_vel_index =
      nq +
      tree->FindChildBodyOfJoint("joint1", model_instance_id2)
          ->get_velocity_start_index();
  int joint2_model1_vel_index =
      nq +
      tree->FindChildBodyOfJoint("joint2", model_instance_id2)
          ->get_velocity_start_index();

  // Simulates the system with initial joint values (2.0,2.0,3.0,4.0) and
  // initial joint velocities (0.1,0.2,0.0,0.0). The trajectory should always
  // almost satisfy the joint transmission constraint.
  x0(joint1_model0_pos_index) = 2.0;
  x0(joint2_model0_pos_index) = 2.0;
  x0(joint1_model1_pos_index) = 3.0;
  x0(joint2_model1_pos_index) = 4.0;
  x0(joint1_model0_vel_index) = 0.1;
  x0(joint2_model0_vel_index) = 0.2;
  x0(joint1_model1_vel_index) = 0.0;
  x0(joint2_model1_vel_index) = 0.0;

  auto trajectory_logger = std::make_shared<
      drake::systems::TrajectoryLogger<RigidBodySystem::StateVector>>(
      rigid_body_system->getNumStates());
  auto sys = drake::cascade(rigid_body_system, trajectory_logger);
  drake::simulate(*sys, kStartTime, duration, x0, options);
  auto trajectory = trajectory_logger->getTrajectory();

  for (size_t i = 0; i < trajectory.time.size(); ++i) {
    EXPECT_NEAR((trajectory.value[i](joint1_model0_pos_index) - 1.0) /
                    trajectory.value[i](joint2_model0_pos_index),
                0.5, 5e-2);
    EXPECT_NEAR((trajectory.value[i](joint1_model1_pos_index) - 1.0) /
                    trajectory.value[i](joint2_model1_pos_index),
                0.5, 5e-2);
  }

  // Simulates the system with initial joint values (2.1,1.9,3.1,4.1) and
  // initial joint velocities (0.11,0.18,0.0,0.02). The initial joint positions
  // and the velocities do not satisfy the transmission constraint. We expect
  // at the end of the simulation, the final position should satisfy the joint
  // transmission constraint
  x0(joint1_model0_pos_index) = 2.1;
  x0(joint2_model0_pos_index) = 1.9;
  x0(joint1_model1_pos_index) = 3.1;
  x0(joint2_model1_pos_index) = 4.1;
  x0(joint1_model0_vel_index) = 0.11;
  x0(joint2_model0_vel_index) = 0.18;
  x0(joint1_model1_vel_index) = 0.0;
  x0(joint2_model1_vel_index) = 0.02;

  trajectory_logger->clearTrajectory();
  drake::simulate(*sys, kStartTime, duration, x0, options);
  trajectory = trajectory_logger->getTrajectory();
  auto final_state = trajectory.value[trajectory.value.size() - 1];
  EXPECT_NEAR((final_state(joint1_model0_pos_index) - 1.0) /
                  final_state(joint2_model0_pos_index),
              0.5, 5e-2);
  EXPECT_NEAR((final_state(joint1_model1_pos_index) - 1.0) /
                  final_state(joint2_model1_pos_index),
              0.5, 5e-2);
}
}  // namespace
}  // namespace mechanical_transmission
}  // namespace examples
}  // namespace drake
