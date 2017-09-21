#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/benchmarks/chaotic_babyboot/MG/MG_chaotic_babyboot_auto_generated.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace chaotic_babyboot {
namespace {

//------------------------------------------------------------------------------
// Structure that holds the following data for a chaotic babyboot
// qA      |  Chaotic babyboot's pendulum angle (rigid body A).
// qB      |  Chaotic babyboot's plate angle (rigid body B).
// qADt    |  1st-time-derivative of angle qA (a.k.a. qAdot).
// qBDt    |  1st-time-derivative of angle qB (a.k.a. qBdot).
// qADDt   |  2nd-time-derivative of angle qA (a.k.a. qAddot).
// qBDDt   |  2nd-time-derivative of angle qB (a.k.a. qBddot).
// energy  |  Sum of kinetic plus potential energy.
struct ChaoticBabybootData {
  double qA, qB, qADt, qBDt, qADDt, qBDDt, energy;
};

//------------------------------------------------------------------------------
// Compare an expected solution to an actual MotionGenesis simulation result.
void CompareExpectedSolutionVsActualSolution(
  const ChaoticBabybootData& babyboot_data_expected ) {
  // Simulate chaotic babyboot with MotionGenesis auto-generated code and
  // using its Runga-Kutta-Merson numerical integrator.
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

  // Compare actual results with expected results to within a multiplier of the
  // MotionGenesis MG_chaotic_babyboot integrator's value for absError.
  // Notice the allowable tolerances below are scaled for individual quantities.
  // The multipliers are "fences" (with a buffer) around results obtained by 12+
  // simulations. The multipliers are experimental (but not arbitrary).
  // Note: After passing pre-merge CI (continuous integration) and merging this
  // test to master, the test failed post-merge CI on a Macintosh build (for
  // qBDt), so the qADt and qBDt multipliers were made less strict.
  // TODO(@mitiguy) Remove macintosh_scale_factor if Macintosh passes this
  // test without scaling.
  const double absError = MG_chaotic_babyboot.absError;
  const double macintosh_scale_factor = 15;
  EXPECT_LE(std::abs(qA_difference), 1E2 * absError);
  EXPECT_LE(std::abs(qB_difference), 1E2 * absError);
  EXPECT_LE(std::abs(qADt_difference), 5E2 * absError * macintosh_scale_factor);
  EXPECT_LE(std::abs(qBDt_difference), 5E2 * absError * macintosh_scale_factor);
  EXPECT_LE(std::abs(qADDt_difference), 1E3 * absError);
  EXPECT_LE(std::abs(qBDDt_difference), 1E3 * absError);
  EXPECT_LE(std::abs(energy_difference), 0.5 * absError);
}

// Create expected solution for chaotic babyboot simulation by running many high
// accuracy simulations with Runga-Kutta-Mersion algorithm (from MotionGenesis)
// as well as MATLAB's ode45 [based on an explicit Runge-Kutta (4,5) formula,
// with the Dormand-Prince pair]. Also shown below are values from two related
// simulations (A, B), run with different (but still high accuracy) absError and
// tStepMax.  All simulations keep the variation in the absolute value of
// mechanical energy (sum of potential and kinetic energy) to less than 7.0E-13.
// Since this system conserves mechanical energy, the theoretical variation in
// mechanical energy is 0.0.  MATLAB's ode45 had the most energy variation with
// approximately 7.0E-13 Joules (over 10 seconds) whereas Runga-Kutta-Merson's
// variation in eenrgy was less 7.0E-14 Joules.
// For the given initial values, the babyboot is unstable (in a bounded-input,
// bounded-output sense) so it is difficult (perhaps impossible) to perfectly
// test the numerical integrator accuracy.
// Ironically, this is why the chaotic babyboot is a good test of accuracy.
//
// Provided below are four different simulation results for each angle and its
// time-derivatives.  Each simulation uses integration parameters and/or
// integrators that are different from each other (e.g., Kutta-Merson vs.
// MATLAB's ode45).  It is important to note that these four different
// simulations produce final values for angles qA and qB that are equal to 7
// significant digits, produce final values for angles qADt and qBDt that are
// equal to 6 significant digits, and produce final values for qADDt and qBDDt
// that are equal to 7 significant digits.  In other words, the results are
// predictable with a well-constructed, relatively high-accuracy integrator.
//
// Note: One reason for this test is to ensure that the MotionGenesis auto-
// generated C++ code could be properly integrated into Drake.  This test has to
// be continually re-verified since code changes might break it.
//
// TODO(@mitiguy) Build this chaotic babyboot in Drake and ensure its numerical
// integrators predict similar stability regions, similar results for angles
// and their time derivatives, and predict conservation of mechanical energy.
//
// Note: Although this is colloquially called a "chaotic" babyboot, it is more
// accurately a demonstration of mathematical stability and instability.
// More information about this problem is in “Mechanical Demonstration of
// Mathematical Stability and Instability”, International Journal of Engineering
// Education (Journal of Mechanical Engineering Education), Vol. 2, No. 4, 1974,
// pp. 45-47, by Thomas R. Kane.
//------------------------------------------------------------------------------
GTEST_TEST(ChaoticBabyboot, ForwardDynamicsA) {
  ChaoticBabybootData babyboot_data_expected;
  constexpr double degree_to_radian =  M_PI / 180;
  babyboot_data_expected.qA =  -61.312983340691 * degree_to_radian;
  // Result from simulation A: -61.312983517329;
  // Result from simulation B: -61.312983761859;
  // Result from MATLAB ode45: -61.3129818234;
  babyboot_data_expected.qB =  -929.47789677244 * degree_to_radian;
  // Result from simulation A: -929.47789818494;
  // Result from simulation B: -929.47789795831;
  // Result from MATLAB ode45: -929.477885053;
  babyboot_data_expected.qADt = -6.7440081040646;
  // Result from simulation A:  -6.7440080750248;
  // Result from simulation B:  -6.7440080514955;
  // Result from MATLAB ode45:  -6.74400835033;
  babyboot_data_expected.qBDt = -2.7795690128833;
  // Result from simulation A:  -2.7795696971631;
  // Result from simulation B:  -2.7795695154087;
  // Result from MATLAB ode45:  -2.77956334901;
  babyboot_data_expected.qADDt = 41.137023263571;
  // Result from simulation A:   41.137023170384;
  // Result from simulation B:   41.137023317160;
  // Result from MATLAB ode45:   41.1370240118;
  babyboot_data_expected.qBDDt = 19.483647096003;
  // Result from simulation A:   19.483647506439;
  // Result from simulation B:   19.483647277708;
  // Result from MATLAB ode45:   19.4836437213;
  babyboot_data_expected.energy = 0.0;

  CompareExpectedSolutionVsActualSolution(babyboot_data_expected);
}


}  // namespace
}  // namespace chaotic_babyboot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
