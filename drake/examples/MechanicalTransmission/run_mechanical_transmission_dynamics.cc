#include "drake/systems/plants/RigidBodySystem.h"
#include <iostream>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/systems/Simulation.h"
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

	auto& tree = rigid_body_system->getRigidBodyTree();
	int nq = tree->number_of_positions();
	int joint1 = tree->findJoint("joint1")->get_position_start_index();
	int joint2 = tree->findJoint("joint2")->get_position_start_index();
	Eigen::Matrix<double,1,Eigen::Dynamic> Aeq_mechanical_transmission(1,nq);
	Aeq_mechanical_transmission.setZero();
	Aeq_mechanical_transmission(0,joint1) = 1.0;
	Aeq_mechanical_transmission(0,joint2) = -0.5;
	tree->addLinearEqualityPositionConstraint(Aeq_mechanical_transmission,
				Eigen::Matrix<double,1,1>::Zero());
	return rigid_body_system;
}

Eigen::VectorXd evaluateMechanicalTransmissionConstraint(const std::shared_ptr<RigidBodyTree> &tree, double joint1_val, double joint2_val) {
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
  auto result1 = evaluateMechanicalTransmissionConstraint(tree,0.0,0.0);
  EXPECT_TRUE(CompareMatrices(result1,Eigen::Matrix<double,1,1>(0.0),Eigen::NumTraits<double>::epsilon(),MatrixCompareType::absolute));

  auto result2 = evaluateMechanicalTransmissionConstraint(tree,1.0,2.0);
  EXPECT_TRUE(CompareMatrices(result2,Eigen::Matrix<double,1,1>(0.0),Eigen::NumTraits<double>::epsilon(),MatrixCompareType::absolute));

  auto result3 = evaluateMechanicalTransmissionConstraint(tree,2.0,2.0);
  EXPECT_FALSE(CompareMatrices(result3,Eigen::Matrix<double,1,1>(0.0),Eigen::NumTraits<double>::epsilon(),MatrixCompareType::absolute));
}
/*
GTEST_TEST(MechanicalTransmissionTest MechanicalTransmissionSimulation) {
	drake::SimulationOptions options;
	options.realtime_factor = 0; // As fast as possible.
	options.initial_step_size = 0.02;

	// Prevents exception from being thrown when simulation runs slower than real
  // time, which it most likely will given the small step size.
  options.warn_real_time_violation = true;

  // Instantates a variable that specifies the duration of the simulation.
  // The default value is 5 seconds.
  double duration = 5.0;

  // Searches through the command line looking for a "--duration" flag followed
  // by a floating point number that specifies a custom duration.
  for (int ii = 1; ii < argc; ++ii) {
    if (std::string(argv[ii]) == "--duration") {
      if (++ii == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--duration\" is not followed by a "
            "value!");
      }
      duration = atof(argv[ii]);
    }
  }

  // Starts the simulation.
  const double kStartTime = 0.0;
  VectorXd x0 = VectorXd::Random(rigid_body_system->getNumStates());
  double stop_time = drake::simulate(*rigid_body_system, kStartTime,
  																	 duration,x0,options);
  std::cout<<"stop time:"<<stop_time<<std::endl;
	return 0;
}
*/
}  // namespace
}  // namespace mechanical_transmission
}  // namespace examples
}  // namespace drake