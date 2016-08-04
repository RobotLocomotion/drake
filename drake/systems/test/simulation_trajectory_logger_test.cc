#include "drake/systems/trajectory_logger.h"

#include "gtest/gtest.h"

#include "drake/systems/Simulation.h"
#include "drake/systems/test/system_test_util.h"

namespace drake {
namespace systems {
namespace test {
namespace {

using drake::EigenVector;

GTEST_TEST(testTrajectoryLogger, testTrajectoryLogger) {
  const int kXdim = 2;
  const int kYdim = 1;
  const int kUdim = 1;
  // simulate a double-integrator xdot = 0*x +[0;u]; y = x(0);
  Eigen::Matrix<double, kXdim, kXdim> A;
  A.setZero();
  A(0, 1) = 1.0;
  Eigen::Matrix<double, kXdim, kUdim> B;
  B.setZero();
  B(1, 0) = 1.0;
  Eigen::Matrix<double, kYdim, kXdim> C;
  C.setZero();
  C(0, 0) = 1.0;
  Eigen::Matrix<double, kYdim, kUdim> D;
  D.setZero();
  Eigen::Matrix<double, kXdim, 1> xdot0;
  xdot0.setZero();
  Eigen::Matrix<double, kYdim, 1> y0;
  y0.setZero();
  auto affine_sys = std::make_shared<
      AffineSystem<EigenVector<kXdim>::type, EigenVector<kUdim>::type,
                   EigenVector<kYdim>::type>>(A, B, xdot0, C, D, y0);

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

  // Starts the simulation.
  const double kStartTime = 0;
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(affine_sys->getNumStates());
  double init_vel = 1.0;
  x0(1) = init_vel;
  drake::simulate(*sys.get(), kStartTime, duration, x0, options);
  auto traj = traj_logger->getTrajectory();
  EXPECT_EQ(traj.time.size(), traj.val.size());
  EXPECT_EQ(traj_logger->getNumOutputs(), static_cast<size_t>(kYdim));
  for (size_t i = 0; i < traj.time.size(); ++i) {
    EXPECT_TRUE(std::abs(traj.val[i](0) - traj.time[i] * init_vel) <= 1E-5);
  }
}

}  // namespace empty
}  // namespace test
}  // namespace systems
}  // namespace drake
