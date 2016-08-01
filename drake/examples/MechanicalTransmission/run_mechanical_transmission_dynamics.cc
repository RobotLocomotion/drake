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
		drake::getDrakePath() +
		"/examples/MechanicalTransmission/mechanical_transmission.urdf",
		DrakeJoint::FIXED);

	rigid_body_system->penetration_stiffness = 3000.0;
	rigid_body_system->penetration_damping = 0;

	auto& tree = rigid_body_system->getRigidBodyTree();
	int nq = tree->number_of_positions();
	int joint1 = tree->findJoint("joint1")->get_position_start_index();
	int joint2 = tree->findJoint("joint2")->get_position_start_index();
	std::cout<<"joint1:"<<joint1<<" joint2:"<<joint2<<std::endl;
	Eigen::Matrix<double,1,Eigen::Dynamic> Aeq_mechanical_transmission(1,nq);
	Aeq_mechanical_transmission.setZero();
	Aeq_mechanical_transmission(0,joint1) = 1.0;
	Aeq_mechanical_transmission(0,joint2) = -0.5;
	tree->addLinearEqualityPositionConstraint(Aeq_mechanical_transmission,
				Eigen::Matrix<double,1,1>::Zero());

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
}
}
}

int main(int argc, char* argv[]) {
	return drake::examples::mechanical_transmission::DoMain(argc, argv);
}