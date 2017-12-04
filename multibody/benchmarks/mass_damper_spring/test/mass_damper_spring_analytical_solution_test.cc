#include "drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace {

// Compare the analytical solution with the expected results.
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// This function compares the analytical solution of a one-degree-of-freedom
// mass-damper-spring system with an expected (special-case) solution at time t.
// m                    |  Mass of system.
// b                    |  Linear damping constant.
// k                    |  Linear spring constant.
// x0                   |  Initial value of x (at time t = 0).
// xDt0                 |  Initial value of ẋ (at time t = 0).
// t                    |  Value of time associated with expected solution.
// x_xDt_xDtDt_expected |  Expected values of x, ẋ, ẍ at time t.
// Note: All units must be self-consistent (e.g., standard SI units).
void CompareMassDamperSpringSolutionVsExpectedSolution(
  const double m, const double b, const double k,
  const double x0, const double xDt0, const double t,
  const Eigen::Vector3d& x_xDt_xDtDt_expected,
  const double tolerance ) {
  // Construct object for calculating analytical solution, set its initial
  // values, and then calculate the analytical solution at time t.
  MassDamperSpringAnalyticalSolution<double> plant(m, b, k);
  plant.SetInitialValue(x0, xDt0);
  const Eigen::Vector3d x_xDt_xDtDt = plant.CalculateOutput(t);

  // Compare the analytical solution with the expected results.
  const bool is_close = x_xDt_xDtDt.isApprox(x_xDt_xDtDt_expected, tolerance);
  EXPECT_TRUE(is_close);
}

// Test accuracy of calculations for undamped free vibration of a simple
// mass-damper-spring system (damping ratio = 0).
GTEST_TEST(MassDamperSpringUndamped, UndampedVibrationA) {
  const double m = 1.0, k = 9.0, b = 0.0;
  const double x0 = 3.0, xDt0 = 0.0;
  const double t = 4.25;

  // Expected special-case solution is x(t) = x0*cos(wn*t).
  const double wn = std::sqrt(k / m);
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << x0 * std::cos(wn * t),
                -x0 * std::sin(wn * t) * wn,
                -x0 * std::cos(wn * t) * wn * wn;

  CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                    x_xDt_xDtDt, 4*kEpsilon);
}

// Test accuracy of calculations for underdamped free vibration of a simple
// mass-damper-spring system (damping ratio = 0.2).
GTEST_TEST(MassDamperSpringUndamped, UnderdampedVibrationB) {
  const double m = 1.0, k = 9.0;
  const double zeta = 0.2,  b = 2 * zeta * std::sqrt(m * k);
  const double x0 = 3.0, xDt0 = 0.0;

  // Expected solution is x(t) = (A*sin(wd*t) + B*cos(wd*t))*exp(-zeta * wn *t).
  // The special case below corresponds to time t = 2*pi/wd.
  const double wn = std::sqrt(k / m);
  const double wd = wn * std::sqrt(1 - zeta * zeta);
  const double t = 2 * M_PI / wd;
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << x0 * std::exp(-zeta * wn * t),
                 0,
                -x0 * wn * wn * std::exp(-zeta * wn * t);

  CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                    x_xDt_xDtDt, 4*kEpsilon);
}

// Test accuracy of calculation for critically-damped free vibration of a simple
// mass-damper-spring system (damping ratio = 1).
GTEST_TEST(MassDamperSpringUndamped, CriticallyDampedC) {
  const double m = 1.0, k = 9.0;
  const double zeta = 1,  b = 2 * zeta * std::sqrt(m * k);
  const double x0 = 0.0, xDt0 = 3.0, t = 0.25;

  // Expected special-case solution is x(t) = ẋ(0) * t * exp(-wn *t).
  const double wn = std::sqrt(k / m);
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << xDt0 * t * std::exp(-wn * t),
                 xDt0 * (1 - wn * t) * std::exp(-wn * t),
                 xDt0 * wn * (-2  + wn * t) * std::exp(-wn * t);

  CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                    x_xDt_xDtDt, 4*kEpsilon);
}

// Test accuracy of calculations for over-damped free vibration of a simple
// mass-damper-spring system (damping ratio = 1.5).
GTEST_TEST(MassDamperSpringUndamped, OverDampedD) {
  const double m = 1.0, k = 9.0;
  const double zeta = 1.5,  b = 2 * zeta * std::sqrt(m * k);
  const double x0 = 0.0, xDt0 = 3.0, t = 0.25;

  // Expected solution was calculated via MotionGenesis.
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << 0.2730433627750083,
                 0.1082082477370706,
                -3.43126449460871;

  CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                    x_xDt_xDtDt, 4*kEpsilon);
}

// Test accuracy of calculation of free vibration of a simple mass-damper-spring
// system at or near critically-damping (damping ratio near 1).
GTEST_TEST(MassDamperSpringUndamped, CriticallyDampedAlmostE) {
  const double m = 1.0, k = 9.0;
  const double x0 = 0.0, xDt0 = 3.0, t = 0.25;

  // For zeta = 1 and x0 = 0, expected solution is x(t) = ẋ(0)*t*exp(-wn*t).
  const double wn = std::sqrt(k / m);
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << xDt0 * t * std::exp(-wn * t),
                 xDt0 * (1 - wn * t) * std::exp(-wn * t),
                 xDt0 * wn * (-2  + wn * t) * std::exp(-wn * t);

  // Test various values for damping ratio near critical damping.
  for (int i = -100; i <= 100;  ++i) {
    const double zeta = 1 + i * kEpsilon;
    const double b = 2 * zeta * std::sqrt(m * k);
    // Tolerance must grow as move away from 1.
    const double min_tolerance = 2E6 * kEpsilon;
    const double max_tolerance = min_tolerance * std::abs(i);
    const double tolerance = std::max(min_tolerance, max_tolerance);
    CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                      x_xDt_xDtDt, tolerance);
  }
}

// Test accuracy of calculations for undamped free vibration of a simple
// mass-damper-spring system (damping ratio = 0) with special initial values,
// namely x(0) = 1 and ẋ(0) = 0.  Also test the partial derivative of the
// output (x, ẋ, ẍ) with respect to mass m.
GTEST_TEST(MassDamperSpringUndamped, UndampedVibrationWithAutoDiffF) {
  AutoDiffXd m, b, k, t;
  m.value() = 1;      m.derivatives() = Vector1d(1);
  b.value() = 0;      b.derivatives() = Vector1d(0);
  k.value() = 9;      k.derivatives() = Vector1d(0);
  t.value() = 4.25;   t.derivatives() = Vector1d(0);

  // Expected special-case solution is x(t) = x0*cos(wn*t).
  using std::sqrt;
  const AutoDiffXd wn = sqrt(k / m);
  Vector3<AutoDiffXd> x_xDt_xDtDt_expected;
  x_xDt_xDtDt_expected << cos(wn * t),
                         -sin(wn * t) * wn,
                         -cos(wn * t) * wn * wn;

  // Construct object for calculating analytical solution, set its initial
  // values, and then calculate the analytical solution at time t.
  AutoDiffXd x0, xDt0;
  xDt0.value() = 0;    xDt0.derivatives() = Vector1d(0);
  x0.value() = 1;        x0.derivatives() = Vector1d(0);
  MassDamperSpringAnalyticalSolution<AutoDiffXd> plant(m, b, k);
  plant.SetInitialValue(x0, xDt0);
  Vector3<AutoDiffXd> x_xDt_xDtDt = plant.CalculateOutput(t);

  // Push AutoDiffXd values into Vector3d (for subsequent comparison).
  Eigen::Vector3d value_plant, value_expected;
  Eigen::Vector3d derivative_plant, derivative_expected;
  for (int i = 0; i < 3; ++i) {
    value_expected(i) = x_xDt_xDtDt_expected(i).value();
    value_plant(i) = x_xDt_xDtDt(i).value();

    derivative_expected(i) = x_xDt_xDtDt_expected(i).derivatives()(0);
    derivative_plant(i) = x_xDt_xDtDt(i).derivatives()(0);
  }

  // Compare plant's solution with simple solution (expected values).
  EXPECT_TRUE(value_plant.isApprox(value_expected, 4 * kEpsilon));

  // Compare partial derivatives with simple solution (expected values).
  EXPECT_TRUE(derivative_plant.isApprox(derivative_expected, 4 * kEpsilon));

  // Compare partial derivatives with respect to m from MotionGenesis (MG).
  // Use 50 * kEpsilon to avoid concerns about copy/paste enough digits.
  // Partial derivatives with respect to m from MotionGenesis.
  Eigen::Vector3d derivative_MG;
  derivative_MG << 1.164069483273479, 19.07735862673189, -1.627938326032988;
  EXPECT_TRUE(derivative_plant.isApprox(derivative_MG, 50 * kEpsilon));
}


}  // namespace
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

