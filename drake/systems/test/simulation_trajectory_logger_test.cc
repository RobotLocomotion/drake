#include "gtest/gtest.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/trajectory_logger.h"
#include "drake/systems/test/system_test_util.h"

using Eigen::Dynamic;

namespace drake {
namespace systems {
namespace test {
namespace {

using drake::EigenVector;

int DoMain(int argc, char* argv[]) {
  const int kXdim = 2;
  const int kYdim = 1;
  const int kUdim = 1;
  // simulate a double-integrator xdot = x +[0;u]; y = x(0);
  Eigen::Matrix<double,kXdim,kXdim> A;
  A.setIdentity();
  Eigen::Matrix<double,kXdim,kUdim> B;
  B.setZero();
  B(1,0) = 1.0;
  Eigen::Matrix<double,kYdim,kXdim> C;
  C.setZero();
  C(0,0) = 1.0;
  Eigen::Matrix<double,kYdim,kUdim> D;
  D.setZero();
  Eigen::Matrix<double,kXdim,1> xdot0;
  xdot0.setZero();
  Eigen::Matrix<double,kYdim,1> y0;
  y0.setZero();
	auto affine_sys = std::make_shared<AffineSystem<
                                       EigenVector<kXdim>::type,
                                       EigenVector<kUdim>::type,
                                       EigenVector<kYdim>::type>
                                    >(A,B,xdot0,C,D,y0);

	auto traj_logger =
	     std::make_shared<TrajectoryLogger<EigenVector<kYdim>::type>>(kYdim);

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
  double init_vel = 1.0;
	x0(1) = init_vel;
  drake::simulate(*sys.get(), kStartTime, duration, x0, options);
  auto traj = traj_logger->getTrajectory();
  if(traj.time.size() != traj.val.size()) {
    throw std::runtime_error("Error: number of time samples does not match "
      "that of the trajectory");
  }
  for(size_t i = 0; i<traj.time.size(); ++i) {
    assert(std::abs(traj.val[i](0)-traj.time[i]*init_vel)<=1E-5);
    assert(std::abs(traj.val[i](1)-init-vel)<=1E-5);
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