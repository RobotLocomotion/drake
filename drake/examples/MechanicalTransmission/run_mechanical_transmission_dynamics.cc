#include <iostream>

#include "drake/Path.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/plants/RigidBodySystem.h"

using drake::RigidBodySystem;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace mechanical_transmission {

int DoMain(int argc, char* argv[]) {
	auto rigid_body_system = std::allocate_shared<RigidBodySystem>(
		Eigen::aligned_allocator<RigidBodySystem>());

	rigid_body_system->addRobotFromFile(
		Drake::getDrakePath() +
		"/examples/MechanicalTransmission/mechanical_transmission.urdf",
		DrakeJoint::FIXED);

	rigid_body_system->penetration_stiffness = 3000.0;
	rigid_body_system->penetration_damping = 0;

	auto& tree = rigid_body_system->getRigidBodyTree();
	int joint1 = tree->findJointId(std::string("joint1"));
	int joint2 = tree->findJointId(std::string("joint2"));
	std::cout<<"joint1:"<<joint1<<" joint2:"<<joint2<<std::endl;
	//tree->addLinearEqualityPositonConstraint
	return 0;
}
}
}
}

int main(int argc, char* argv[]) {
	return drake::examples::mechanical_transmission::DoMain(argc, argv);
}