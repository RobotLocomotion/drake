#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/benchmarks/chaotic_babyboot/MG/MG_chaotic_babyboot_auto_generated.h"

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
// their time-derivatives to expected solution that was created by running a
// very high accuracy simulation with MotionGenesis - that kept energy variation
// to less than -1.2656542480727E-14 (the theoretical difference is 0.0).
GTEST_TEST(ChaoticBabyboot, ForwardDynamicsA) {
  const double degree_to_radian =  0.0174532925199432957692369;
  const double qA_expected = -61.312983517329 * degree_to_radian;
  const double qB_expected = -929.47789818494 * degree_to_radian;
  const double qADt_expected = -6.7440080750248;
  const double qBDt_expected = -2.7795696971631;
  const double qADDt_expected =  41.137023170384;
  const double qBDDt_expected = 19.483647506439;
  const double energy_expected = 0.0;

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

