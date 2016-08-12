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

std::shared_ptr<RigidBodySystem> parseMechanicalTransmission() {
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

Eigen::VectorXd evaluateMechanicalTransmissionConstraint(
    const std::shared_ptr<RigidBodyTree>& tree, double joint1_val,
    double joint2_val) {
  int nq = tree->number_of_positions();
  int joint1 = tree->findJoint("joint1")->get_position_start_index();
  int joint2 = tree->findJoint("joint2")->get_position_start_index();
  VectorXd q0(nq);
  q0(joint1) = joint1_val;
  q0(joint2) = joint2_val;
  auto kinematics_cache = tree->doKinematics(q0);
  auto result = tree->positionConstraints(kinematics_cache);
  return result;
}
// Test parsing mechanical transmission from urdf
GTEST_TEST(MechanicalTransmissionTest, ParseMechanicalTransmission) {
  auto rigid_body_system = parseMechanicalTransmission();
  auto tree = rigid_body_system->getRigidBodyTree();
  auto result1 = evaluateMechanicalTransmissionConstraint(tree, 1.0, 0.0);
  std::cout << "result1 is " << result1.rows() << " x " << result1.cols()
            << std::endl;
  std::cout << result1(0) << std::endl;
  EXPECT_TRUE(CompareMatrices(result1, Eigen::Matrix<double, 1, 1>(0.0),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));

  auto result2 = evaluateMechanicalTransmissionConstraint(tree, 2.0, 2.0);
  std::cout << result2(0) << std::endl;
  EXPECT_TRUE(CompareMatrices(result2, Eigen::Matrix<double, 1, 1>(0.0),
                              Eigen::NumTraits<double>::epsilon(),
                              MatrixCompareType::absolute));

  auto result3 = evaluateMechanicalTransmissionConstraint(tree, 2.0, 3.0);
  std::cout << result3(0) << std::endl;
  EXPECT_FALSE(CompareMatrices(result3, Eigen::Matrix<double, 1, 1>(0.0),
                               Eigen::NumTraits<double>::epsilon(),
                               MatrixCompareType::absolute));
}

GTEST_TEST(MechanicalTransmissionTest, MechanicalTransmissionSimulation) {
  auto rigid_body_system = parseMechanicalTransmission();
  auto tree = rigid_body_system->getRigidBodyTree();
  int nq = tree->number_of_positions();
  drake::SimulationOptions options;
  options.realtime_factor = 0;  // As fast as possible.
  options.initial_step_size = 0.02;

  // Prevents exception from being thrown when simulation runs slower than real
  // time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;

  // Instantates a variable that specifies the duration of the simulation.
  // The default value is 5 seconds.
  double duration = 5.0;

  // Starts the simulation.
  const double kStartTime = 0.0;
  VectorXd x0 = VectorXd::Zero(rigid_body_system->getNumStates());
  int joint1_pos = tree->findJoint("joint1")->get_position_start_index();
  int joint2_pos = tree->findJoint("joint2")->get_position_start_index();
  int joint1_vel = nq + tree->findJoint("joint1")->get_velocity_start_index();
  int joint2_vel = nq + tree->findJoint("joint2")->get_velocity_start_index();

  x0(joint1_pos) = 2.0;
  x0(joint2_pos) = 2.0;
  x0(joint1_vel) = 0.1;
  x0(joint2_vel) = 0.2;
  auto trajectory_logger = std::make_shared<
      drake::systems::TrajectoryLogger<RigidBodySystem::StateVector>>(
      rigid_body_system->getNumStates());
  auto sys = drake::cascade(rigid_body_system, trajectory_logger);
  drake::simulate(*sys, kStartTime, duration, x0, options);
  auto trajectory = trajectory_logger->getTrajectory();

  for (size_t i = 0; i < trajectory.time.size(); ++i) {
    EXPECT_NEAR((trajectory.value[i](joint1_pos) - 1.0) /
                    trajectory.value[i](joint2_pos),
                0.5, 5e-2);
  }
  x0(joint1_pos) = 2.0;
  x0(joint2_pos) = 2.5;
  x0(joint1_vel) = 0.1;
  x0(joint2_vel) = 0.18;
  trajectory_logger->clearTrajectory();
  drake::simulate(*sys, kStartTime, duration, x0, options);
  trajectory = trajectory_logger->getTrajectory();
  auto final_state = trajectory.value[trajectory.value.size() - 1];
  EXPECT_NEAR((final_state(joint1_pos) - 1.0) / final_state(joint2_pos), 0.5,
              5e-2);
}

}  // namespace
}  // namespace mechanical_transmission
}  // namespace examples
}  // namespace drake
