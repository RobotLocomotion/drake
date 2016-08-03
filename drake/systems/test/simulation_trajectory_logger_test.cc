#include "gtest/gtest.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/TrajectoryLogger.h"
#include "drake/systems/test/system_test_util.h"

using Eigen::Dynamic;

namespace drake {
namespace systems {
namespace test {
namespace {

using drake::EigenVector;

int DoMain(int argc, char* argv[]) {
	auto affine_sys = drake::system_test::CreateRandomAffineSystem<2,2,2>(
		2, 2, 2);
  //auto sys1 = AffineSystem<EigenVector<2>::type,EigenVector<2>::type,EigenVector<2>::type>();
  //TrajectoryLogger<EigenVector<2>::type> logger();

	auto traj_logger =
	     std::make_shared<TrajectoryLogger<EigenVector<2>::type>>(2);

	auto sys = cascade(affine_sys, traj_logger);

	// Specifies the simulation options.
  drake::SimulationOptions options;
  options.realtime_factor = 0;  // As fast as possible.
  options.initial_step_size = 0.002;

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
	const double kStartTime = 0;
	Eigen::VectorXd x0 = Eigen::VectorXd::Zero(affine_sys->getNumStates());
	x0(0) = 1.0;
  drake::simulate(*sys.get(), kStartTime, duration, x0, options);
  auto traj_time = traj_logger->getTrajectoryTime();
  auto traj_val = traj_logger->getTrajectorySamples();
  if(traj_time.size() != traj_val.size()) {
    throw std::runtime_error("Error: number of time samples does not match "
      "that of the trajectory");
  }
  return 0;
}

} // namespace empty
} // namespace test
} // namespace systems
} // namespace drake

int main(int argc, char* argv[]) {
  return drake::systems::test::DoMain(argc, argv);
}