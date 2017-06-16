#include "drake/multibody/benchmarks/mass_damper_spring/mass_damper_spring_analytical_solution.h"

#include <cmath>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace {

// Compare the analytical solution with the expected results.
constexpr double epsilon = std::numeric_limits<double>::epsilon();

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
  const double m = 1.0, b = 0.0, k = 4.0 * M_PI * M_PI;
  const double x0 = 3.0, xDt0 = 0.0;
  const double t = 4.25;

  // Expected special-case solution is x(t) = x0*cos(wn*t).
  const double wn = std::sqrt(k / m);
  Eigen::Vector3d x_xDt_xDtDt;
  x_xDt_xDtDt << x0 * std::cos(wn * t),
                -x0 * std::sin(wn * t) * wn,
                -x0 * std::cos(wn * t) * wn * wn;

  CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                    x_xDt_xDtDt, 4*epsilon);
}

// Test accuracy of calculations for underdamped free vibration of a simple
// mass-damper-spring system (damping ratio = 0.2).
GTEST_TEST(MassDamperSpringUndamped, UnderdampedVibrationB) {
  const double m = 1.0, k = 4.0 * M_PI * M_PI;
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
                                                    x_xDt_xDtDt, 4*epsilon);
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
                                                    x_xDt_xDtDt, 4*epsilon);
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
                                                    x_xDt_xDtDt, 4*epsilon);
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
  for (int i = -100; i <= 100;  i++) {
    const double zeta = 1 + i * epsilon;
    const double b = 2 * zeta * std::sqrt(m * k);
    // Tolerance must grow as move away from 1.
    const double min_tolerance = 2E6 * epsilon;
    const double max_tolerance = min_tolerance * std::abs(i);
    const double tolerance = std::max(min_tolerance, max_tolerance);
    CompareMassDamperSpringSolutionVsExpectedSolution(m, b, k, x0, xDt0, t,
                                                      x_xDt_xDtDt, tolerance);
  }
}

// Test accuracy of calculations for undamped free vibration of a simple
// mass-damper-spring system (damping ratio = 0).  Also test the partial
// derivative of the output (x, ẋ, ẍ) with respect to mass m.
GTEST_TEST(MassDamperSpringUndamped, UndampedVibrationWithAutoDiffF) {
  AutoDiffXd m, b, k, x0, xDt0, t;
  m.value() = 1;                m.derivatives() = Vector1d(1);
  b.value() = 0;                b.derivatives() = Vector1d(0);
  k.value() = 4 * M_PI * M_PI;  k.derivatives() = Vector1d(0);
  x0.value() = 3;               x0.derivatives() = Vector1d(0);
  xDt0.value() = 0;             xDt0.derivatives() = Vector1d(0);
  t.value() = 4.25;             t.derivatives() = Vector1d(0);

  // Expected special-case solution is x(t) = x0*cos(wn*t).
  using std::sqrt;
  const AutoDiffXd wn = sqrt(k / m);
  Vector3<AutoDiffXd> x_xDt_xDtDt_expected;
  x_xDt_xDtDt_expected << x0 * cos(wn * t),
                         -x0 * sin(wn * t) * wn,
                         -x0 * cos(wn * t) * wn * wn;

  // Construct object for calculating analytical solution, set its initial
  // values, and then calculate the analytical solution at time t.
  MassDamperSpringAnalyticalSolution<AutoDiffXd> plant(m, b, k);
  plant.SetInitialValue(x0, xDt0);
  Vector3<AutoDiffXd> x_xDt_xDtDt = plant.CalculateOutput(t);

  // Partial derivatives with respect to m from MotionGenesis.
  Eigen::Vector3d partial_derivatives_with_respect_to_m;
  partial_derivatives_with_respect_to_m << 40.05530633326987,
                                            9.424777960768763,
                                        -1581.320110695291;

  // Compare the analytical solution with the expected results.
  Eigen::Vector3d values_expected, derivatives_expected;
  for (int i = 0; i < 3; i++) {
    const double value_expected = x_xDt_xDtDt_expected(i).value();
    const double value_plant = x_xDt_xDtDt(i).value();
    const double value_difference = value_expected - value_plant;
    const bool is_value_close = std::abs(value_difference) < 4 * epsilon;
    EXPECT_TRUE(is_value_close);

    // Compare partial derivatives with simple solution (expected values).
    const double derivative_expected = x_xDt_xDtDt_expected(i).derivatives()(0);
    const double derivative_plant = x_xDt_xDtDt(i).derivatives()(0);
    double derivative_difference = derivative_expected - derivative_plant;
    bool is_derivative_close = std::abs(derivative_difference) < 4 * epsilon;
    EXPECT_TRUE(is_derivative_close);

    // Compare partial derivatives with results from MotionGenesis (MG).
    const double derivative_MG = partial_derivatives_with_respect_to_m(i);
    derivative_difference = derivative_MG - derivative_plant;
    is_derivative_close = std::abs(derivative_difference) < 5000 * epsilon;
    EXPECT_TRUE(is_derivative_close);
  }
}


}  // namespace
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

