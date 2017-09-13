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

//------------------------------------------------------------------------------
// Structure that holds the following data for a chaotic babyboot
// qA      |  Chaotic babyboot's pendulum angle (rigid body A).
// qB      |  Chaotic babyboot's plate angle (rigid body B).
// qADt    |  1st-time-derivative of angle qA (a.k.a. qAdot).
// qADt    |  1st-time-derivative of angle qB (a.k.a. qBdot).
// qADDt   |  2nd-time-derivative of angle qA (a.k.a. qAddot).
// qBDDt   |  2nd-time-derivative of angle qB (a.k.a. qBddot).
// energy  |  Sum of kinetic plus potential energy.
struct ChaoticBabybootData {
  double qA, qB, qADt, qBDt, qADDt, qBDDt, energy;
};


void CompareExpectedSolutionVsActualSolution(
  const ChaoticBabybootData& babyboot_data_expected ) {
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

  // Calculate the difference betwqeen actual and expected results.
  const double qA_difference = qA - babyboot_data_expected.qA;
  const double qB_difference = qB - babyboot_data_expected.qB;
  const double qADt_difference = qADt - babyboot_data_expected.qADt;
  const double qBDt_difference = qBDt - babyboot_data_expected.qBDt;
  const double qADDt_difference = qADDt - babyboot_data_expected.qADDt;
  const double qBDDt_difference = qBDDt - babyboot_data_expected.qBDDt;
  const double energy_difference = energy - babyboot_data_expected.energy;

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
  EXPECT_TRUE(std::abs(energy_difference) <= 0.1 * absError);
}

// Test accuracy of calculations for chaotic babyboot simulation.
// Create expected solution by running three high accuracy simulations with
// Runga-Kutta-Mersion algorithm, all keeping the absolute value of energy
// variation to less than 6.0E-14 (theoretical is 0.0).
// For the given initial values, the babyboot is chaotic, so it is difficult
// (perhaps impossible) to perfectly test the numerical integrator accuracy.
// Ironically, this is why the chaotic babyboot is a good test of accuracy.
GTEST_TEST(ChaoticBabyboot, ForwardDynamicsA) {
  ChaoticBabybootData babyboot_data_expected;
  constexpr double degree_to_radian =  0.0174532925199432957692369;
  babyboot_data_expected.qA =  -61.312983340691 * degree_to_radian;
  // Result from simulation A: -61.312983517329;
  // Result from simulation B: -61.312983761859
  babyboot_data_expected.qB =  -929.47789677244 * degree_to_radian;
  // Result from simulation A: -929.47789818494;
  // Result from simulation B: -929.47789795831;
  babyboot_data_expected.qADt = -6.7440081040646;
  // Result from simulation A:  -6.7440080750248;
  // Result from simulation B:  -6.7440080514955;
  babyboot_data_expected.qBDt = -2.7795690128833;
  // Result from simulation A:  -2.7795696971631;
  // Result from simulation B:  -2.7795695154087;
  babyboot_data_expected.qADDt = 41.137023263571;
  // Result from simulation A:   41.137023170384;
  // Result from simulation B:   41.137023317160;
  babyboot_data_expected.qBDDt = 19.483647096003;
  // Result from simulation A:   19.483647506439;
  // Result from simulation B:   19.483647277708;
  babyboot_data_expected.energy = 0.0;

  CompareExpectedSolutionVsActualSolution(babyboot_data_expected);
}


}  // namespace
}  // namespace MG
}  // namespace chaotic_babyboot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
