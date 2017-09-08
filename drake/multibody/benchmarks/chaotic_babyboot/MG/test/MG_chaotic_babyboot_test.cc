#include "drake/multibody/benchmarks/chaotic_babyboot/MG/MG_chaotic_babyboot_auto_generated.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace chaotic_babyboot {
namespace MG {
namespace {

using Vector7d = Eigen::Matrix<double, 7, 1>;

// Function to compare chaotic babyboot's angles qA and qB and their time-
// derivatives to expected (known textbook) solutions.
// qA_expected        |  Chaotic babyboot's pendulum angle (rigid body A).
// qB_expected        |  Chaotic babyboot's plate angle (rigid body B).
// qADt_expected      |  1st-time-derivative of angle qA (a.k.a. qAdot).
// qADt_expected      |  1st-time-derivative of angle qB (a.k.a. qBdot).
// qADDt_expected     |  2nd-time-derivative of angle qA (a.k.a. qAddot).
// qBDDt_expected     |  2nd-time-derivative of angle qB (a.k.a. qBddot).
// energy_expected    |  Sum of kinetic plus potential energy.
void CompareExpectedSolutionVsActualSolution(
  const double qA_expected,
  const double qB_expected,
  const double qADt_expected,
  const double qBDt_expected,
  const double qADDt_expected,
  const double qBDDt_expected,
  const double energy_expected) {
  // Simulate chaotic babyboot.
  MotionGenesis::MGChaoticBabyboot_::MGChaoticBabyboot MG_chaotic_babyboot;
  MG_chaotic_babyboot.MGSimulate();
  const double qA = MG_chaotic_babyboot.qA;
  const double qB = MG_chaotic_babyboot.qB;
  const double qADt = MG_chaotic_babyboot.qAp;
  const double qBDt = MG_chaotic_babyboot.qBp;
  const double qADDt = MG_chaotic_babyboot.qApp;
  const double qBDDt = MG_chaotic_babyboot.qBpp;
  const double energy = MG_chaotic_babyboot.Energy;

  // Compare actual results with expected results.
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  const double tolerance = 1.0E7 * kEpsilon;
  Vector7d expected, actual;
  expected << qA_expected, qB_expected,
              qADt_expected, qBDt_expected,
              qADDt_expected, qBDDt_expected, energy_expected;
  actual << qA, qB, qADt, qBDt, qADDt, qBDDt < energy;
  EXPECT_TRUE(actual.isApprox(expected, tolerance));
}


// Test accuracy of calculations for chaotic babyboot's angles qA and qB and
// their time-derivatives to expected (textbook) solution.
GTEST_TEST(ChaoticBabyboot, ForwardDynamicsA) {
  const double qA_expected = -6.131271E+01;
  const double qB_expected = -9.294758E+02;
  const double qADt_expected = -6.744052E+00;
  const double qBDt_expected = -2.778562E+00;
  const double qADDt_expected = 4.113716E+01;
  const double qBDDt_expected = 1.948305E+01;
  const double energy_expected = -5.515546E-11;

  CompareExpectedSolutionVsActualSolution(qA_expected, qB_expected,
                                          qADt_expected, qBDt_expected,
                                          qADDt_expected, qBDDt_expected,
                                          energy_expected);
}


}  // namespace
}  // namespace MG
}  // namespace chaotic_babyboot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake

