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

  // Calculate the absolute value of the difference in results.
  const double qA_difference = qA - qA_expected;
  const double qB_difference = qB - qB_expected;
  const double qADt_difference = qADt - qADt_expected;
  const double qBDt_difference = qBDt - qBDt_expected;
  const double qADDt_difference = qADDt - qADDt_expected;
  const double qBDDt_difference = qBDDt - qBDDt_expected;
  const double energy_difference = energy - energy_expected;

  // Compare actual results with expected results to within a
  // multiplier of the integrator's value of absError.
  // Allowable error is scaled for individual quantities.
  const double absError = MG_chaotic_babyboot.absError;
  EXPECT_TRUE(std::abs(qA_difference) <= 1E2 * absError);
  EXPECT_TRUE(std::abs(qB_difference) <= 1E2 * absError);
  EXPECT_TRUE(std::abs(qADt_difference) <= 5E2 * absError);
  EXPECT_TRUE(std::abs(qBDt_difference) <= 5E2 * absError);
  EXPECT_TRUE(std::abs(qADDt_difference) <= 1E3 * absError);
  EXPECT_TRUE(std::abs(qBDDt_difference) <= 1E3 * absError);
  EXPECT_TRUE(std::abs(energy_difference) <= 0.01*absError);
}


// Test accuracy of calculations for chaotic babyboot's angles qA and qB and
// their time-derivatives.  Compare to expected solution that was created by
// running two very high accuracy simulation with MotionGenesis - both kept the
// absolute value of energy variation to less than 6.0E-14 (theoretical is 0.0).
// For the given initial values, the babyboot is chaotic, so it is difficult
// (perhaps impossible) to create a perfect test that integrator works.
GTEST_TEST(ChaoticBabyboot, ForwardDynamicsA) {
  const double degree_to_radian =  0.0174532925199432957692369;
  const double qA_expected = -61.312983761859 * degree_to_radian;
  // Other simulation:       -61.312983517329;
  const double qB_expected = -929.47789795831 * degree_to_radian;
  // Other simulation:       -929.47789818494;
  const double qADt_expected = -6.7440080514955;
  // Other simulation:         -6.7440080750248;
  const double qBDt_expected = -2.7795695154087;
  // Other simulation:         -2.7795696971631;
  const double qADDt_expected = 41.137023317160;
  // Other simulation:          41.137023170384;
  const double qBDDt_expected = 19.483647277708;
  // Other simulation:          19.483647506439;
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
