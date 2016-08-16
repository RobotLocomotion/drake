#include <fstream>
#include <iostream>
#include "drake/systems/plants/RigidBodySystem.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/trajectory_logger.h"
#include "drake/util/eigen_matrix_compare.h"

using drake::RigidBodySystem;
using Eigen::VectorXd;
using drake::util::CompareMatrices;
using drake::util::MatrixCompareType;

namespace drake {
namespace examples {
namespace mechanical_transmission {
namespace {
/**
 * Loads mechanical_transmission.urdf, which says that
 * joint1_value = 0.5*joint2_value+1.0,
 * and returns the RigidBodySystem.
 */
std::shared_ptr<RigidBodySystem> ParseMechanicalTransmission() {
  auto rigid_body_system = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  rigid_body_system->AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/examples/MechanicalTransmission/mechanical_transmission.urdf",
      DrakeJoint::FIXED);

  rigid_body_system->penetration_stiffness = 3000.0;
  rigid_body_system->penetration_damping = 0;
  return rigid_body_system;
}

/**
 * Evaluates the constraint joint1_value - 0.5*joint2_value-1.0, to check if the
 * mechanical transmission imposes the right constraint through URDF parsing.
 */
Eigen::VectorXd EvaluateMechanicalTransmissionConstraint(
    const std::shared_ptr<RigidBodyTree>& tree, double joint1_val,
    double joint2_val, int model_id = -1) {
  int nq = tree->number_of_positions();
  int joint1_idx =
      tree->findJoint("joint1", model_id)->get_position_start_index();
  int joint2_idx =
      tree->findJoint("joint2", model_id)->get_position_start_index();
  std::cout << "joint1_idx:" << joint1_idx << " joint2_idx:" << joint2_idx
            << std::endl;
  VectorXd q0(nq);
  q0.setZero();
  q0(joint1_idx) = joint1_val;
  q0(joint2_idx) = joint2_val;
  auto kinematics_cache = tree->doKinematics(q0);
  return tree->positionConstraints(kinematics_cache);
}

// Tests the ability to parse a URDF mechanical transmission specification.
GTEST_TEST(MechanicalTransmissionTest, ParseMechanicalTransmission) {
  auto rigid_body_system = ParseMechanicalTransmission();
  auto tree = rigid_body_system->getRigidBodyTree();

  // Joints values (1.0, 0.0) should satisfy the constraint, check if the offset
  // is set correctly.
  Eigen::VectorXd result1 =
      EvaluateMechanicalTransmissionConstraint(tree, 1.0, 0.0);
  EXPECT_TRUE(CompareMatrices(result1, Eigen::Matrix<double, 1, 1>(0.0),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Joints values (2.0, 2.0) should satisfy the constraint.
  Eigen::VectorXd result2 =
      EvaluateMechanicalTransmissionConstraint(tree, 2.0, 2.0);
  EXPECT_TRUE(CompareMatrices(result2, Eigen::Matrix<double, 1, 1>(0.0),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Joints values (2.0, 3.0) should not satisfy the constraint.
  Eigen::VectorXd result3 =
      EvaluateMechanicalTransmissionConstraint(tree, 2.0, 3.0);
  EXPECT_TRUE(CompareMatrices(result3, Eigen::Matrix<double, 1, 1>(-0.5),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Test the simulation with the mechanical transmission.
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
  auto rigid_body_system = ParseMechanicalTransmission();
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
  int joint1_pos_index = tree->findJoint("joint1")->get_position_start_index();
  int joint2_pos_index = tree->findJoint("joint2")->get_position_start_index();
  int joint1_vel_index =
      nq + tree->findJoint("joint1")->get_velocity_start_index();
  int joint2_vel_index =
      nq + tree->findJoint("joint2")->get_velocity_start_index();

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

// Test if the joint transmission constraint is parsed correctly when we have
// two identical model instances in the same rigid body tree.
GTEST_TEST(MechanicalTransmissionTest,
           ParseMechanicalTransmissionMultipleModelInstance) {
  auto rigid_body_system = ParseMechanicalTransmission();
  rigid_body_system->AddModelInstanceFromFile(
      drake::GetDrakePath() +
          "/examples/MechanicalTransmission/mechanical_transmission.urdf",
      DrakeJoint::FIXED);
  auto tree = rigid_body_system->getRigidBodyTree();
  std::cout << "number of model instances"
            << tree->get_number_of_model_instances() << std::endl;

  // Joint values (2.0,2.0,0.0,0.0) should satisfy the first joint transmission
  // constraint
  Eigen::VectorXd result1 =
      EvaluateMechanicalTransmissionConstraint(tree, 2.0, 2.0, 0);
  Eigen::Vector2d result1_check(0.0, -1.0);
  EXPECT_TRUE(CompareMatrices(result1, result1_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));

  // Joint values (0.0,0.0,2.0,0.0) should satisfy the second joint transmission
  // constraint
  Eigen::VectorXd result2 =
      EvaluateMechanicalTransmissionConstraint(tree, 2.0, 2.0, 1);
  Eigen::Vector2d result2_check(-1.0, 0.0);
  EXPECT_TRUE(CompareMatrices(result2, result2_check,
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));
}
}  // namespace
}  // namespace mechanical_transmission
}  // namespace examples
}  // namespace drake
