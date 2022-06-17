// Purpose: Compare Drake with the exact (closed-form) solution from Section
//          1.13 (pgs. 60-62) of Spacecraft Dynamics, by Kane & Levinson.
//          This closed-form solution includes both angular velocity and
//          Euler parameters for an axis-symmetric rigid body B, when the moment
//          of forces on B about Bcm (B's center of mass) is zero (torque-free).
//-----------------------------------------------------------------------------
#include <cmath>
#include <tuple>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/free_body/free_body.h"
#include "drake/multibody/test_utilities/floating_body_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace free_body {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Quaterniond;
using multibody::test::AxiallySymmetricFreeBodyPlant;
using systems::Simulator;

const double kEpsilon = std::numeric_limits<double>::epsilon();

// This function tests Drake solution versus an exact (closed-form) solution
// for the torque-free motion of an axis-symmetric uniform rigid cylinder B in
// a Newtonian frame N, where torque-free means the moment of forces about
// Bcm (B's mass center) is zero.
// @param[in] torque_free_cylinder_exact class that stores initial values,
//            gravity, and methods to calculate the exact solution at time t.
// @param[in] axisymmetric_plant Drake's rigid-body system being simulated.
// @param[in] context all input to System (parameters, time, state, etc.,)
// @param[out] stateDt_drake stores time-derivative values of state at time t.
// @param[in] tolerance minimum tolerance required to match results.
void TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
       const FreeBody& torque_free_cylinder_exact,
       const AxiallySymmetricFreeBodyPlant<double>& axisymmetric_plant,
       const drake::systems::Context<double>& context,
       drake::systems::ContinuousState<double>* stateDt_drake,
       const double tolerance) {
  DRAKE_DEMAND(stateDt_drake != NULL);

  // Get the state at time t from the context and pull out meaningful results.
  const systems::VectorBase<double>& state_drake =
      context.get_continuous_state_vector();
  const VectorXd state_as_vector = state_drake.CopyToVector();
  const Vector4d quat4_NB_drake = state_as_vector.segment<4>(0);
  const Vector3d xyz_drake      = state_as_vector.segment<3>(4);
  const Vector3d w_NB_N_drake   = state_as_vector.segment<3>(7);
  const Vector3d v_NBcm_N_drake = state_as_vector.segment<3>(10);
  const Eigen::Quaterniond quat_NB_drake(quat4_NB_drake(0), quat4_NB_drake(1),
                                         quat4_NB_drake(2), quat4_NB_drake(3));

  // Calculate the time-derivative of the state and pull out meaningful results.
  // quatDt_NB is the time-derivative of quat_NB, i.e., [ė0  ė1  ė2  ė3].
  // xyzDt is [ẋ, ẏ, ż] Bcm's velocity in N, expressed in N, i.e., the time-
  //           derivative of [x, y, z] (Bcm's position from No, expressed in N).
  // wDt_NB_N  is B's angular acceleration in N, expressed in N, and
  //           is equal to [ẇx, ẇy, ẇz], where [wx, wy, wz] is w_NB_N.
  // xyzDDt is [ẍ  ÿ  z̈] Bcm's acceleration in N, expressed in N.
  axisymmetric_plant.CalcTimeDerivatives(context, stateDt_drake);
  const VectorXd stateDt_as_vector = stateDt_drake->CopyToVector();
  const Vector4d quatDt_NB_drake = stateDt_as_vector.segment<4>(0);
  const Vector3d xyzDt_drake     = stateDt_as_vector.segment<3>(4);
  const Vector3d wDt_NB_N_drake  = stateDt_as_vector.segment<3>(7);
  const Vector3d xyzDDt_drake    = stateDt_as_vector.segment<3>(10);
  EXPECT_TRUE(CompareMatrices(v_NBcm_N_drake, xyzDt_drake, kEpsilon));

  // Calculate exact (closed-form) rotational solution at time t.
  const double t = context.get_time();
  Quaterniond quat_NB_exact;
  Vector4d quatDt_NB_exact;
  Vector3d w_NB_B_exact, wDt_NB_B_exact;
  std::tie(quat_NB_exact, quatDt_NB_exact, w_NB_B_exact, wDt_NB_B_exact) =
      torque_free_cylinder_exact.CalculateExactRotationalSolutionNB(t);

  // Calculate exact (closed-form) translational solution at time t,
  // specifically, p_NoBcm_N, v_NBcm_N, a_NBcm_N.
  Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;
  std::tie(xyz_exact, xyzDt_exact, xyzDDt_exact) =
      torque_free_cylinder_exact.CalculateExactTranslationalSolution(t);

  // Since more than one quaternion is associated with the same orientation,
  // compare Drake's canonical quaternion with exact canonical quaternion.
  const bool is_ok_quat = math::AreQuaternionsEqualForOrientation(
                          quat_NB_exact, quat_NB_drake, tolerance);
  EXPECT_TRUE(is_ok_quat);

  // Convert quaternions to rotation matrix and also compare rotation matrices.
  const math::RotationMatrixd R_NB_drake(quat_NB_drake);
  const math::RotationMatrixd R_NB_exact(quat_NB_exact);
  EXPECT_TRUE(R_NB_drake.IsNearlyEqualTo(R_NB_exact, tolerance));

  // Compare Drake and exact results.
  const Vector3d w_NB_B_drake = R_NB_drake.inverse() * w_NB_N_drake;
  const Vector3d wDt_NB_B_drake = R_NB_drake.inverse() * wDt_NB_N_drake;
  EXPECT_TRUE(CompareMatrices(w_NB_B_drake,     w_NB_B_exact,  2*tolerance));
  EXPECT_TRUE(CompareMatrices(wDt_NB_B_drake, wDt_NB_B_exact,  8*tolerance));
  EXPECT_TRUE(CompareMatrices(xyz_drake,           xyz_exact,    tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDt_drake,       xyzDt_exact,    tolerance));
  EXPECT_TRUE(CompareMatrices(v_NBcm_N_drake,    xyzDt_exact,    tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDDt_drake,     xyzDDt_exact,  8*tolerance));

  // Multi-step process to compare Drake vs exact time-derivative of quaternion.
  // Ensure time-derivative of Drake's quaternion satisfies quaternionDt test.
  // Since more than one time-derivative of a quaternion is associated with the
  // same angular velocity, convert to angular velocity to compare results.
  EXPECT_TRUE(math::IsBothQuaternionAndQuaternionDtOK(
      quat_NB_drake, quatDt_NB_drake, 16*tolerance));

  EXPECT_TRUE(math::IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB(
      quat_NB_exact, quatDt_NB_exact, w_NB_B_exact, 16*tolerance));

  EXPECT_TRUE(math::IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB(
      quat_NB_drake, quatDt_NB_drake, w_NB_B_drake, 16*tolerance));
}


// Numerically integrate rigid body plant's equations of motion from time t = 0
// to time t_final with a 3rd-order variable-step Runge-Kutta integrator.
// @param[in] torque_free_cylinder_exact class that stores initial values,
//            gravity, and methods to calculate the exact solution at time t.
// @param[in] axisymmetric_plant Drake's rigid-body system being simulated.
// @param[in,out] context on entry, context should be properly filled with all
// input to system (parameters, time = 0.0, state).  On output, context is
// changed so its time = t_final and its state has been updated.
// @param[out] stateDt_drake on output, stores time-derivatives of state.
// @param[in] dt_max maximum (fixed) step size taken by the integrator.
// @param[in] t_final value of time that signals the simulation to stop.
// t_final is not required to be an exact multiple of dt_max as the
// integration step is adjusted as necessary to land at exactly t_final.
// @param[in] number_of_periods number of periods of the sinusoidal motion
// being simulated.  Allowable error scales linearly with this value.
void  IntegrateForwardWithVariableStepRungeKutta3(
          const FreeBody& torque_free_cylinder_exact,
          const AxiallySymmetricFreeBodyPlant<double>& axisymmetric_plant,
          drake::systems::Context<double>* context,
          drake::systems::ContinuousState<double>* stateDt_drake,
          const double dt_max, const double t_final,
          const int number_of_periods) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL  &&
               dt_max > 0.0  &&  t_final > 0.0);

  // Integrate with variable-step Runge-Kutta3 integrator.
  systems::RungeKutta3Integrator<double> rk3(axisymmetric_plant, context);
  rk3.set_maximum_step_size(dt_max);  // Need before Initialize (or exception).
  const double maximum_absolute_error_per_integration_step = 1.0E-3;
  rk3.set_target_accuracy(maximum_absolute_error_per_integration_step);
  rk3.Initialize();

  // TODO(Mitiguy) Replace the code below with Evan's StepOnceExactly integrator
  // method once feature request (issue #5581) request for the integrator
  // counter is in place - which allows testing of StepOnceExactly to ensure
  // it works as well (or probably better) than this temporary solution.

  // Integrate to within a small amount of dt of t_final.
  const double epsilon_based_on_dt_max = 1.0E-11 * dt_max;
  const double t_final_minus_epsilon = t_final - epsilon_based_on_dt_max;
  while (true) {
    // At the boundary of each numerical integration step, test Drake's results
    // versus exact (closed-form) solution.  Based on numerical integration
    // experiments, the numerical integration seems to scale linearly with time,
    // so the allowable error tolerance below is scaled linearly with time.
    // Note: The numerical integration experiments were done by modifying the
    // MotionGenesis script (see webpage below) so that its inertia parameters
    // and initial values matched this problems parameters/initial values.
    // The numerical integrator used by MotionGenesis is Runga-Kutta-Merson,
    // described at L. Fox, Numerical Solutions of Ordinary and Partial
    // Differential Equations, Palo Alto: Addison-Wesley, 1962, pp. 24-25
    // http://www.motiongenesis.com/MGWebSite/MGGetStarted/MGExampleSpinStability3DRigidBody/MGExampleSpinStability3DRigidBody.html
    const double t = context->get_time();
    const double integrator_value = 256;  // Specific to each integrator.
    const double scale = number_of_periods * integrator_value * t / t_final;
    const double tolerance =
        (1 + scale) * maximum_absolute_error_per_integration_step;
    TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
        torque_free_cylinder_exact,
        axisymmetric_plant,
        *context,
        stateDt_drake,
        tolerance);

    // Check if simulation is finished.
    if (t >= t_final_minus_epsilon) break;

    // Step forward by at most dt.
    const double t_max = std::min(t + dt_max, t_final);
    rk3.IntegrateNoFurtherThanTime(t_max, t_max, t_max);
  }
}


// This function tests Drake's calculation of the time-derivative of the state
// at an initial condition, and depending on should_numerically_integrate, also
// tests Drake's numerical integration of the rigid body's equations of motion.
// @param[in] torque_free_cylinder_exact class that stores initial values,
//            gravity, and methods to calculate the exact solution at time t.
// @param[in] axisymmetric_plant Drake's rigid-body system being simulated.
// @param[in,out] context on entry, context is properly sized for all input to
// system (parameters, time = 0.0, state).  On output, context may be changed
// changed so its time = t_final and its state has been updated.
// @param[out] stateDt_drake on output, stores time-derivatives of state.
// @param[in] should_numerically_integrate set to `true` if calling function
// wants this function to also numerically integrate and check accuracy of
// Drake's simulation results.
void  TestDrakeSolutionForSpecificInitialValue(
    const FreeBody& torque_free_cylinder_exact,
    const AxiallySymmetricFreeBodyPlant<double>& axisymmetric_plant,
    drake::systems::Context<double>* context,
    drake::systems::ContinuousState<double>* stateDt_drake,
    const bool should_numerically_integrate) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL);

  // Set Drake's state (context) from initial values by concatenating
  // 4 smaller Eigen column matrices into one larger Eigen column matrix.
  //
  // Drake's quaternion joint state is e0,e1,e2,e3, x,y,z, wx,wy,wz, vx,vy,vz.
  //
  // e0, e1, e2, e3 is the quaternion quat_NB relating unit vectors Nx, Ny, Nz
  // fixed in N (World) to unit vectors Bx, By, Bz fixed in cylinder B.  Bz is
  // directed along B's symmetric axis. e0 is the scalar part of the quaternion.
  //
  // x, y, z is p_NoBcm_N, the position from No (World origin) to Bcm (B's
  // center of mass) expressed in N, where Bcm is the cylinder's centroid.
  //
  // wx, wy, wz is w_NB_N, B's angular velocity in N, expressed in N.
  //
  // vx, vy, vz is v_NBcm_N, the velocity of Bcm in N, expressed in N.
  const Quaterniond& quat_NB = torque_free_cylinder_exact.get_initial_quat_NB();
  const Vector3d p_NoBcm_N = torque_free_cylinder_exact.get_initial_p_NoBcm_N();
  const Vector3d w_NB_N = torque_free_cylinder_exact.CalcInitial_w_NB_N();
  const Vector3d v_NBcm_N = torque_free_cylinder_exact.CalcInitial_v_NBcm_N();
  Eigen::Matrix<double, 13, 1> state_initial;
  state_initial << quat_NB.w(), quat_NB.x(), quat_NB.y(), quat_NB.z(),
                   p_NoBcm_N, w_NB_N, v_NBcm_N;
  systems::VectorBase<double>& state_drake =
      context->get_mutable_continuous_state_vector();
  state_drake.SetFromVector(state_initial);

  // Ensure the time stored by context is set to 0.0 (initial value).
  context->SetTime(0.0);

  // Test Drake's calculated values for the time-derivative of the state at
  // time t = 0 versus the exact (closed-form) solution.
  // Initially (t = 0), Drake's results should be close to machine-precision.
  TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
      torque_free_cylinder_exact,
      axisymmetric_plant, *context, stateDt_drake,
      32 * kEpsilon);

  // Maybe numerically integrate.
  if (should_numerically_integrate) {
    // Calculate the two "pseudo" frequencies associated with this motion.  Use
    // the maximum of these two frequencies to approximate the time period of
    // one full cycle of motion.  Use this to determine the length of time to
    // integrate such that we capture four periods of motion, or if the maximum
    // frequency is too small, ten seconds of time.
    double s, p;
    std::tie(s, p) = torque_free_cylinder_exact.CalcAngularRates_s_p();
    const double frequency = std::max(kEpsilon, std::max(std::abs(s), p));
    const double period = 2 * M_PI / frequency;
    const int number_of_periods = 4;
    const double t_final = std::max(number_of_periods * period, 10.0);

    // To decide a maximum integration step, it seemed a reasonably "good" sine
    // curve resulted from 17 data points (16 intervals).
    const double reasonable_number_of_intervals_per_period = 16.0;
    const double dt_max = period / reasonable_number_of_intervals_per_period;

    // Integrate forward, testing Drake's results vs. exact solution frequently.
    IntegrateForwardWithVariableStepRungeKutta3(
        torque_free_cylinder_exact,
        axisymmetric_plant, context, stateDt_drake,
        dt_max, t_final, number_of_periods);
  }
}


// This function sets up various initial values for Drake to test against an
// exact solution. For one (mostly arbitrary) initial value, this also tests
// Drake's numerical integration of the rigid body's equations of motion.
// @param[in] torque_free_cylinder_exact class that stores initial values,
//            gravity, and methods to calculate the exact solution at time t.
// @param[in] axisymmetric_plant rigid-body system being simulated.
void  TestDrakeSolutionForVariousInitialValues(
      FreeBody* torque_free_cylinder_exact,
      const AxiallySymmetricFreeBodyPlant<double>& axisymmetric_plant) {
  DRAKE_DEMAND(torque_free_cylinder_exact != nullptr);

  // Drake's Simulator will create a Context by calling axisymmetric_plant's
  // CreateDefaultContext(). This in turn will initialize its state by making a
  // call to this system's SetDefaultState().
  systems::Simulator<double> simulator(axisymmetric_plant);
  systems::Context<double>& context = simulator.get_mutable_context();

  // Allocate space to hold the time-derivative of the Drake state.
  std::unique_ptr<systems::ContinuousState<double>> stateDt =
      axisymmetric_plant.AllocateTimeDerivatives();
  drake::systems::ContinuousState<double>* stateDt_drake = stateDt.get();

  // Test a variety of initial normalized quaternions.
  // Since cylinder B is axis-symmetric for axis Bz, iterate on BodyXY rotation
  // sequence with 0 <= thetaX <= 2*pi and 0 <= thetaY <= pi.
  int test_counter = 0;
  for (double thetaX = 0; thetaX <= 2*M_PI; thetaX += 0.1*M_PI) {
    for (double thetaY = 0; thetaY <= 0.5*M_PI; thetaY += 0.1*M_PI) {
      const math::RollPitchYaw<double> spaceXYZ_angles(thetaX, thetaY, 0);
      const Quaterniond initial_quat_NB = spaceXYZ_angles.ToQuaternion();
      torque_free_cylinder_exact->set_initial_quat_NB(initial_quat_NB);

      // Since there are 20 * 5 = 100 total tests of initial conditions, only
      // perform time-consuming numerically integration infrequently.
      const bool also_numerically_integrate = (test_counter++ == 120);
      TestDrakeSolutionForSpecificInitialValue(*torque_free_cylinder_exact,
                                               axisymmetric_plant,
                                               &context, stateDt_drake,
                                               also_numerically_integrate);
    }
  }
}


// This function tests Drake's simulation of the motion of an axis-symmetric
// rigid body B (uniform cylinder) in a Newtonian frame (World) N.  Since the
// only external forces on B are uniform gravitational forces, the moment of
// all forces about B's mass center is zero ("torque-free"), there exists an
// exact (closed-form) solution for B's motion.  Specifically, this
// function tests Drake's solution against an exact solution for: quaternion,
// angular velocity expressed in B (body-frame), angular acceleration expressed
// in B and position, velocity, and acceleration expressed in N (World).
// Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
//
// - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
//  (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
//   https://ecommons.cornell.edu/handle/1813/637
GTEST_TEST(uniformSolidCylinderTorqueFree, testNumericalIntegration) {
  // Store initial values in a class that can calculate an exact solution.
  // Store gravitational acceleration expressed in N (e.g. [0, 0, -9.81]).
  const Quaterniond quat_NB(1, 0, 0, 0);    // Initial value.
  const Vector3d w_NB_B(2.0, 4.0, 6.0);     // Initial value.
  const Vector3d p_NoBcm_N(1.0, 2.0, 3.0);  // Initial value.
  const Vector3d v_NBcm_B(-4.2, 5.5, 6.1);  // Initial value.
  const Vector3d gravity(0, 0, -9.81);      // Per note below, in -Nz direction.
  FreeBody torque_free_cylinder_exact(quat_NB, w_NB_B, p_NoBcm_N, v_NBcm_B,
                                      gravity);

  // Instantiate the Drake model for the free body in space.
  // Note the Drake model requires gravity to be in the -Nz direction.
  const double mass = 1.0;  // Arbitrary value.
  const double inertia_transverse = torque_free_cylinder_exact.get_I();
  const double inertia_axial = torque_free_cylinder_exact.get_J();
  AxiallySymmetricFreeBodyPlant<double> axisymmetric_plant(
     mass, inertia_transverse, inertia_axial, -gravity(2));

  TestDrakeSolutionForVariousInitialValues(&torque_free_cylinder_exact,
                                           axisymmetric_plant);
}

// This helper function checks Kane's exact solution against an expected simple
// solution for torque-free motion of an axisymmetric rigid body B.  The
// expected simple solution is associated with simple spin about Bz (B's inertia
// symmetry axis) or simple spin about some combination of Bx and By.  With
// these restrictions, the equation Torque = I*theta'' is applicable where
// Torque = 0 and theta'' is the second time derivative of the angle theta.
// @param[in] torque_free_cylinder_kane class that stores initial values,
//            gravity, and methods to calculate Kane's solution at time t.
// @param[in] thetaDt body B's initial angular spin rate (radians/second).
// @param[in] lambda unit vector in B's initial angular velocity direction,
//            which must be either Bz, -Bz, or a combination of Bx and By.
void TestKaneExactSolution(FreeBody torque_free_cylinder_kane,
                           const double thetaDt, const Vector3d& lambda) {
  ASSERT_NE(thetaDt, 0);  // Ensure B's angular velocity is non-zero.

  // Ensure lambda is a unit vector and its Bz component is 1, -1, or 0.
  ASSERT_TRUE(std::abs(1 - lambda.norm()) <= 4 * kEpsilon);
  const bool is_simple_Bz_spin = std::abs(1 - std::abs(lambda(2))) < 4*kEpsilon;
  ASSERT_TRUE(is_simple_Bz_spin || lambda(2) == 0);

  // As described above, the governing ODE for this test is theta'' = 0.
  // Knowing the initial value of theta is theta(t = 0) = 0, this ODE's solution
  // is theta' = constant and theta(t) = theta' * t.
  const double t = 1.23;             // Arbitrary value of time.
  const double theta = thetaDt * t;  // Solution to ODE.
  const Vector3d w_expected = thetaDt * lambda;
  const Vector3d wDt_expected(0, 0, 0);

  // theta-lambda defines this quaternion [Kane, pg. 12 1983] by
  const double q0 = std::cos(theta/2);
  const double q1 = lambda(0) * std::sin(theta/2);
  const double q2 = lambda(1) * std::sin(theta/2);
  const double q3 = lambda(2) * std::sin(theta/2);

  // Time-differentiating the previous expressions for q0, q1, q2, q3 gives
  // q0Dt, q1Dt, q2Dt, q3Dt.  Note: Since this test employs simple 2D spin,
  // lambda(i) (i = 0, 1, 2) are constant.  In general 3D rotational motion,
  // lambda(i) (i = 0, 1, 2) are not constant.
  const double q0Dt = -0.5 * std::sin(theta/2) * thetaDt;
  const double q1Dt = 0.5 * lambda(0) * std::cos(theta/2) * thetaDt;
  const double q2Dt = 0.5 * lambda(1) * std::cos(theta/2) * thetaDt;
  const double q3Dt = 0.5 * lambda(2) * std::cos(theta/2) * thetaDt;

  // Package the previous expected results for subsequent comparison with Kane.
  const Quaterniond quat_expected(q0, q1, q2, q3);
  const Vector4d quatDt_expected(q0Dt, q1Dt, q2Dt, q3Dt);

  // Get Kane's general 3D solution for B's given initial angular velocity.
  torque_free_cylinder_kane.set_initial_w_NB_B(w_expected);
  Quaterniond quat_kane;
  Vector4d quatDt_kane;
  Vector3d w_kane, wDt_kane;
  std::tie(quat_kane, quatDt_kane, w_kane, wDt_kane) =
      torque_free_cylinder_kane.CalculateExactRotationalSolutionNB(t);

  // Ensure Kane's general 3D solution matches this 2D specialized solution.
  // To that end, compare Kane's angular velocity/acceleration and Kane's
  // quaternion and its time-derivative to their expected results.

  // Ensure Kane's angular velocity/accelerations match the expected ones.
  EXPECT_TRUE(CompareMatrices(w_kane,     w_expected,  4 * kEpsilon));
  EXPECT_TRUE(CompareMatrices(wDt_kane, wDt_expected,  4 * kEpsilon));

  // Since more than one quaternion is associated with the same orientation,
  // compare Kane's quaternion with the expected quaternion.
  const bool is_ok_quat = math::AreQuaternionsEqualForOrientation(
      quat_kane, quat_expected, 8 * kEpsilon);
  EXPECT_TRUE(is_ok_quat);

  // Ensure Kane's quaternion time-derivative matches expected angular velocity.
  EXPECT_TRUE(math::IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB(
    quat_kane, quatDt_kane, w_expected, 8 * kEpsilon));

  // Ensure expected quaternion time-derivative matches Kane's angular velocity.
  EXPECT_TRUE(math::IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB(
      quat_expected, quatDt_expected, w_kane, 8 * kEpsilon));
}


// This test helps verify the analytical solution proposed by [Kane, 1983] for
// the torque-free 3D rotational motion of an axisymmetric rigid body B (e.g., a
// uniform cylinder) in a Newtonian frame (World) N.  The initial values used in
// this test make it easy to compute an associated 2D solution for simple spin.
// Kane's analytical 3D solution depends on intermediate calculations from the
// FreeBody method CalcAngularRates_s_p().  Hence, this test also helps
// verify the underlying FreeBody method CalcAngularRates_s_p().
GTEST_TEST(uniformSolidCylinderTorqueFree, testKaneExactSolution) {
  const Quaterniond quat_NB(1, 0, 0, 0);  // Initial value -- aligned axes.
  const Vector3d w_NB_B(0, 0, 0);         // Initial value -- overwritten.
  const Vector3d p_NoBcm_N(0, 0, 0);      // Initial value.
  const Vector3d v_NBcm_B(0, 0, 0);       // Initial value.
  const Vector3d gravity(0, 0, 0);        // No gravity.
  FreeBody torque_free_cylinder_kane(quat_NB, w_NB_B, p_NoBcm_N, v_NBcm_B,
                                     gravity);

  double thetaDt = 4.56;         // Spin rate.
  Vector3d lambda(1, 0, 0);      // Test spin about Bx.
  TestKaneExactSolution(torque_free_cylinder_kane, thetaDt, lambda);

  lambda = Vector3d(0, 1, 0);    // Test spin about By.
  TestKaneExactSolution(torque_free_cylinder_kane, thetaDt, lambda);

  lambda = Vector3d(0, 0, 1);    // Test spin about Bz.
  TestKaneExactSolution(torque_free_cylinder_kane, thetaDt, lambda);

  const double xy = 1.0/std::sqrt(2);
  lambda = Vector3d(xy, xy, 0);  // Test spin about (Bx + By) / sqrt(2).
  TestKaneExactSolution(torque_free_cylinder_kane, thetaDt, lambda);
}

}  // namespace
}  // namespace free_body
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
