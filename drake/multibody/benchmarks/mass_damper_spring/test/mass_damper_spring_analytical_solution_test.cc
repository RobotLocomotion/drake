#include "drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h"

#include <cmath>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace {

// Function to compare solution from mass-damper-spring system with
// expected solution at time t,
// m                    |  Mass of system.
// b                    |  Linear damping constant (translational damper).
// k                    |  Linear spring constant (translational spring).
// x0                   |  Initial value of x (at time t = 0).
// xDt0                 |  Initial value of x' (at time t = 0).
// t                    |  Value of time associated with expected solution.
// x_xDt_xDtDt_expected |  Expected values of x, x', x'' at time t.
void CompareMassDamperSpringSolutionVsExpectedSolution(
  const double m, const double b, const double k,
  const double x0, const double xDt0, const double t,
  const Eigen::Vector3d& x_xDt_xDtDt_expected ) {
  // Construct object for calculating analytical solution.
  MassDamperSpringAnalyticalSolution plant(m, b, k);

  // Set the initial values for calculating the analytical solution.
  plant.SetInitialValue(x0, xDt0);

  // Calculate the analytical solution at time t.
  const Eigen::Vector3d x_xDt_xDtDt = plant.CalculateOutput(t);

  // Compare the analytical solution with expected results.
  constexpr double epsilon = std::numeric_limits<double>::epsilon();
  const double tolerance = 10 * epsilon;
  for (int i = 0; i < 3; i++) {
  std::cout << "\n ";
  std::cout << x_xDt_xDtDt(i);
  std::cout << "  BLAH    ";
  std::cout << x_xDt_xDtDt_expected(i);
  std::cout << "\n";
  }

  EXPECT_TRUE(x_xDt_xDtDt.isApprox(x_xDt_xDtDt_expected, tolerance));
}


// Test accuracy of calculations for undamped free vibration of a system
// comprised of a classical mass-damper-spring system when damping = 0.
GTEST_TEST(MassDamperSpringUndamped, UndampedVibrationA) {
  const double m = 1.0, b = 0.0, k = 4.0 * M_PI * M_PI;
  const double x0 = 3.0, xDt0 = 0.0;
  const double t = 4.25;

  // Expected solution is x(t) = x0*cos(wn*t).
  const double wn = std::sqrt(k / m);
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << x0 * cos(wn * t),
                -x0 * sin(wn * t) * wn,
                -x0 * cos(wn * t) * wn * wn;

  // Compare this expected solution with the more general solution available
  // in the class MassDamperSpringAnalyticalSolution.
  CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                    x_xDt_xDtDt);
}

// Test accuracy of calculations for undamped free vibration of a system
// comprised of a classical mass-damper-spring system when damping ratio = 0.2
GTEST_TEST(MassDamperSpringUndamped, UnderdampedVibrationB) {
  const double m = 1.0, b = 1.0, k = 4.0 * M_PI * M_PI;
  const double x0 = 3.0, xDt0 = 0.0;

  // Expected solution is x(t) = (A*sin(wd*t) + B*cos(wd*t))*exp(-zeta * wn *t).
  // Special case result for time t = 2*pi/wd.
  const double wn = std::sqrt(k / m);
  const double zeta = b / (2 * std::sqrt(m * k));
  const double wd = wn * std::sqrt(1 - zeta * zeta);
  const double t = 2 * M_PI / wd;
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << x0 * std::exp(-zeta * wn * t),
                 0,
                -x0 * wn * wn * std::exp(-zeta * wn * t);

  // Compare this expected solution with the more general solution available
  // in the class MassDamperSpringAnalyticalSolution.
  CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                    x_xDt_xDtDt);
}


}  // namespace
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

