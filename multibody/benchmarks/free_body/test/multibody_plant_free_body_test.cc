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
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/test/floating_body_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
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
// @param[in] maximum_absolute_error_per_integration_step maximum allowable
// absolute error (for any variable being integrated) for the numerical
// integrator's internal time-step (which can shrink or grow).
void  IntegrateForwardWithVariableStepRungeKutta3(
          const FreeBody& torque_free_cylinder_exact,
          const AxiallySymmetricFreeBodyPlant<double>& axisymmetric_plant,
          drake::systems::Context<double>* context,
          drake::systems::ContinuousState<double>* stateDt_drake,
          const double dt_max, const double t_final,
          const double maximum_absolute_error_per_integration_step) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL  &&  dt_max >= 0.0);

  // Integrate with variable-step Runge-Kutta3 integrator.
  systems::RungeKutta3Integrator<double> rk3(axisymmetric_plant, context);
  rk3.set_maximum_step_size(dt_max);  // Need before Initialize (or exception).
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
    // At boundary of each numerical integration step, test Drake's simulation
    // accuracy versus exact (closed-form) solution.
    const double tolerance = 4 * maximum_absolute_error_per_integration_step;
    TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
        torque_free_cylinder_exact,
        axisymmetric_plant,
        *context,
        stateDt_drake,
        tolerance);

    const double t = context->get_time();
    if (t >= t_final_minus_epsilon) break;

    const double dt = (t + dt_max > t_final) ? (t_final - t) : dt_max;
    rk3.IntegrateAtMost(dt, dt, dt);    // Step forward by at most dt.
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
  context->set_time(0.0);

  // Test Drake's calculated values for the time-derivative of the state at
  // time t = 0 versus the exact (closed-form) solution.
  // Initially (t = 0), Drake's results should be close to machine-precision.
  TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
      torque_free_cylinder_exact,
      axisymmetric_plant, *context, stateDt_drake,
      32 * kEpsilon);

  // Maybe numerically integrate.
  if (should_numerically_integrate) {
    const double dt_max = 0.2, t_final = 10.0;
    const double maximum_absolute_error_per_integration_step = 1.0E-3;

    // Integrate forward, testing Drake's results vs. exact solution frequently.
    IntegrateForwardWithVariableStepRungeKutta3(
        torque_free_cylinder_exact,
        axisymmetric_plant, context, stateDt_drake,
        dt_max, t_final, maximum_absolute_error_per_integration_step);
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
GTEST_TEST(uniformSolidCylinderTorqueFree, testA) {
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


}  // namespace
}  // namespace free_body
}  // namespace benchmarks
}  // namespace drake
