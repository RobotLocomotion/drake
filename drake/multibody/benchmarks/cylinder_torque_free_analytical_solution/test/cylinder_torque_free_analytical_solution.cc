// Purpose: Compare Drake with analytical closed-form solution from Section
//          1.13 (pgs. 60-62) of Spacecraft Dynamics, by Kane & Levinson.
//          This closed-form solution includes both angular velocity and
//          Euler parameters for an axis-symmetric rigid body B, when the moment
//          of forces on B about Bcm (B's center of mass) is zero (torque-free).
//-----------------------------------------------------------------------------
#include <cmath>
#include <tuple>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace benchmarks {
namespace cylinder_torque_free_analytical_solution {
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Quaterniond;


/**
 * Calculates exact solutions for quaternion, angular velocity, and angular
 * acceleration expressed in body-frame, for torque-free rotational motion of an
 * axis-symmetric rigid body B (uniform cylinder) in Newtonian frame (World) A,
 * where torque-free means the moment of forces about B's mass center is zero.
 * Right-handed orthogonal unit vectors Ax, Ay, Az fixed in A are initially
 * equal to right-handed orthogonal unit vectors Bx, By, Bz fixed in B,
 * where Bz is parallel to B's symmetry axis.
 * Note: The function CalculateExactRotationalSolutionNB() is a more general
 * solution that allows for initial misalignment of Bx, By, Bz.
 * Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 * @param t Current value of time.
 * @param w_NB_B_initial  B's initial angular velocity in N, expressed in B.
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat_AB    | Quaternion relating Ax, Ay, Az to Bx, By, Bz.
 *            | Note: quat_AB is analogous to the rotation matrix R_AB.
 * w_NB_B     | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
 * wDt_NB_B   | B's angular acceleration in N, expressed in B, [wx', wy', wz'].
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
std::tuple<Quaterniond, Vector3d, Vector3d>
CalculateExactRotationalSolutionABInitiallyAligned(const double t,
                                 const Vector3d& w_NB_B_initial) {
  // Constant values of moments of inertia.
  const double I = 0.04;
  const double J = 0.02;

  // Initial values of wx, wy, wz.
  const double wx0 = w_NB_B_initial[0];
  const double wy0 = w_NB_B_initial[1];
  const double wz0 = w_NB_B_initial[2];

  // Intermediate calculations for quaternion solution.
  const double p = std::sqrt(wx0*wx0 + wy0*wy0 + (wz0 * J / I) * (wz0 * J / I));
  const double s = (I - J) / I * wz0;
  const double coef = wz0 * (J / (I * p));
  const double spt2 = std::sin(p * t / 2);
  const double cpt2 = std::cos(p * t / 2);
  const double sst2 = std::sin(s * t / 2);
  const double cst2 = std::cos(s * t / 2);

  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is a set Ax, Ay, Az of right-handed orthogonal unit vectors which
  // are fixed in N (Newtonian frame/World) and initially aligned to Bx, By, Bz,
  // where Bz is parallel to B's symmetry axis.
  // Kane produced an analytical solution for quat_AB [eAB0, eAB1, eB2, eB3],
  // which allows for direct calculation of the R_AB rotation matrix.
  const double eAB1 = spt2 / p * (wx0 * cst2 + wy0 * sst2);
  const double eAB2 = spt2 / p * (-wx0 * sst2 + wy0 * cst2);
  const double eAB3 =  coef * spt2 * cst2 + cpt2 * sst2;
  const double eAB0 = -coef * spt2 * sst2 + cpt2 * cst2;
  const Quaterniond quat_AB(eAB0, eAB1, eAB2, eAB3);

  // Analytical solution for wx(t), wy(t), wz(t).
  const double wx =  wx0 * std::cos(s * t) + wy0 * std::sin(s * t);
  const double wy = -wx0 * std::sin(s * t) + wy0 * std::cos(s * t);
  const double wz =  wz0;

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt =  (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;

  // Create a tuple to package for returning.
  std::tuple<Quaterniond, Vector3d, Vector3d> returned_tuple;
  std::get<0>(returned_tuple) = quat_AB;
  Vector3d& w_NB_B   = std::get<1>(returned_tuple);
  Vector3d& wDt_NB_B = std::get<2>(returned_tuple);

  // Fill returned_tuple with rotation results.
  w_NB_B   << wx, wy, wz;
  wDt_NB_B << wxDt, wyDt, wzDt;

  return returned_tuple;
}


/**
 * Calculates exact solutions for quaternion and angular velocity expressed in
 * body-frame, and their time derivatives for torque-free rotational motion of
 * axis-symmetric rigid body B (uniform cylinder) in Newtonian frame (World) N,
 * where torque-free means the moment of forces about B's mass center is zero.
 * The quaternion characterizes the orientiation between right-handed orthogonal
 * unit vectors Nx, Ny, Nz fixed in N and right-handed orthogonal unit vectors
 * Bx, By, Bz fixed in B, where Bz is parallel to B's symmetry axis.
 * Note: CalculateExactRotationalSolutionABInitiallyAligned() implements the
 * algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 * This function allows for initial misalignment of Nx, Ny, Nz and Bx, By, Bz.
 * @param t Current value of time.
 * @param quat_NB_initial Initial value of the quaternion (which should already
 *   be normalized) that relates Nx, Ny, Nz to Bx, By, Bz.
 *   Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
 * @param w_NB_B_initial  B's initial angular velocity in N, expressed in B.
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat_NB    | Quaternion relating Nx, Ny, Nz to Bx, By, Bz: [e0, e1, e2, e3].
 *            | Note: quat_NB is analogous to the rotation matrix R_NB.
 * quatDt     | Time-derivative of `quat_NB', i.e., [e0', e1', e2', e3'].
 * w_NB_B     | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
 * wDt_NB_B   | B's angular acceleration in N, expressed in B, [wx', wy', wz'].
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d>
   CalculateExactRotationalSolutionNB(const double t,
                                      const Quaterniond& quat_NB_initial,
                                      const Vector3d& w_NB_B_initial) {
  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is another set Ax, Ay, Az of right-handed orthogonal unit vectors
  // which are fixed in N (Newtonian frame) but initially aligned to Bx, By, Bz.
  Quaterniond quat_AB;
  Vector3d w_NB_B, wDt_NB_B;
  std::tie(quat_AB, w_NB_B, wDt_NB_B) =
      CalculateExactRotationalSolutionABInitiallyAligned(t, w_NB_B_initial);

  // Multiply (Hamilton product) the quaternions analogous to the rotation
  // matrices R_NA and R_AB to produce the quaternion characterizing R_NB.
  // In other words, account for quat_NB_initial (which is quat_NA) to calculate
  // the quaternion quat_NB that is analogous to the R_NB rotation matrix.

  // Define A basis as World-fixed basis aligned with B's initial orientation.
  const Quaterniond& quat_NA = quat_NB_initial;
  const Quaterniond quat_NB = quat_NA * quat_AB;

  // Analytical solution for time-derivative quaternion in B.
  const Vector4d quatDt =
  math::CalculateQuaternionDtFromAngularVelocityExpressedInB(quat_NB, w_NB_B);

  // Create a tuple to package for returning.
  std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d> returned_tuple;
  std::get<0>(returned_tuple) = quat_NB;
  std::get<1>(returned_tuple) = quatDt;
  std::get<2>(returned_tuple) = w_NB_B;
  std::get<3>(returned_tuple) = wDt_NB_B;

  return returned_tuple;
}


/**
 * Calculates exact solutions for translational motion of an arbitrary rigid
 * body B in a Newtonian frame (world) N.  Algorithm from high-school physics.
 * @param t Current value of time.
 * @param xyz_initial Initial values of x, y, z -- [the Nx, Ny, Nz measures of
 * Bcm's (B's center of mass) position from a point No fixed in World N].
 * Note: Nx, Ny, Nz are right-handed orthogonal unit vectors fixed in N.
 * @param xyzDt_initial Initial values of x', y', z' (which are time-derivatives
 * of x, y,z -- and equal to the Nx, Ny, Nz measures of Bcm's velocity in N).
 * @param gravity Gravitational acceleration expressed in N (e.g. [0, 0, -9.8]).
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-----------------------------------------------------------
 * xyz        | Vector3d x, y, z (Bcm's position from No, expressed in N).
 * xyzDt      | Vector3d with first-time-derivative of x, y, z.
 * xyzDDt     | Vector3D with second-time-derivative of x, y, z.
 */
std::tuple<Vector3d, Vector3d, Vector3d>
    CalculateExactTranslationalSolution(const double t,
                                        const Vector3d& xyz_initial,
                                        const Vector3d& xyzDt_initial,
                                        const Vector3d& gravity) {
  // Initial values of x, y, z and x', y', z'.
  const double x0 = xyz_initial[0];
  const double y0 = xyz_initial[1];
  const double z0 = xyz_initial[2];
  const double xDt0 = xyzDt_initial[0];
  const double yDt0 = xyzDt_initial[1];
  const double zDt0 = xyzDt_initial[2];

  // Analytical solution for x'', y'', z'' (only gravitational forces on body).
  const double x_acceleration = gravity[0];
  const double y_acceleration = gravity[1];
  const double z_acceleration = gravity[2];

  // Analytical solution for x', y', z' (constant acceleration).
  const double xDt = xDt0 + x_acceleration * t;
  const double yDt = yDt0 + y_acceleration * t;
  const double zDt = zDt0 + z_acceleration * t;

  // Analytical solution for x, y, z (constant acceleration).
  const double x = x0 + xDt0*t + 0.5 * x_acceleration * t * t;
  const double y = y0 + yDt0*t + 0.5 * y_acceleration * t * t;
  const double z = z0 + zDt0*t + 0.5 * z_acceleration * t * t;

  // Create a tuple to package for returning.
  std::tuple<Vector3d, Vector3d, Vector3d> returned_tuple;
  Vector3d& xyz    = std::get<0>(returned_tuple);
  Vector3d& xyzDt  = std::get<1>(returned_tuple);
  Vector3d& xyzDDt = std::get<2>(returned_tuple);

  // Fill returned_tuple with rotation results.
  xyz    << x, y, z;
  xyzDt  << xDt, yDt, zDt;
  xyzDDt << x_acceleration, y_acceleration, z_acceleration;

  return returned_tuple;
}


/**
 * This function tests Drake state versus an exact solution for torque-free
 * motion of axis-symmetric uniform rigid cylinder B in Newtonian frame/World N,
 * where torque-free means the moment of forces about B's mass center is zero.
 * The quaternion characterizes the orientiation between right-handed orthogonal
 * unit vectors Nx, Ny, Nz fixed in N and right-handed orthogonal unit vectors
 * Bx, By, Bz fixed in B, where Bz is parallel to B's symmetry axis.
 * @param rigid_body_plant A reference to the rigid-body system being simulated.
 * @param[in] context All input to System (input ports, parameters, time, state)
 * with cache for computations dependent on these values.
 * @param[out] stateDt_drake On output, stores value at t = t_final.
 * @param quat_NB_initial Initial value of the quaternion (which should already
 *   be normalized) that relates Nx, Ny, Nz to Bx, By, Bz.
 *   Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
 * @param w_NB_B_initial  B's initial angular velocity in N, expressed in B.
 * @param xyz_initial Initial values of x, y, z -- [the Nx, Ny, Nz measures of
 * Bcm's (B's center of mass) position from a point No fixed in N].
 * @param v_NBo_B_initial Initial values of Bx, By, Bz measures of Bo's velocity
 * in N, where Bo is the same as Bcm.  Note: These values are not (in general)
 * x', y', z' (i.e., these values are not time-derivatives of x, y,z).
 * @param tolerance Minimum tolerance required to match results.
 */
  void TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
       const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
       const drake::systems::Context<double>& context,
       drake::systems::ContinuousState<double>* stateDt_drake,
       const Quaterniond& quat_NB_initial,
       const Vector3d& w_NB_B_initial,
       const Vector3d& xyz_initial,
       const Vector3d& v_NBo_B_initial,
       const double tolerance) {
  DRAKE_DEMAND(stateDt_drake != NULL);

  // Pull the state at time t from the context.
  const systems::VectorBase<double>& state_drake =
      context.get_continuous_state_vector();

  // Evaluate the time-derivatives of the state.
  rigid_body_plant.CalcTimeDerivatives(context, stateDt_drake);

  // Get the state (defined above) and state time-derivatives for comparison.
  const VectorXd state_as_vector = state_drake.CopyToVector();
  const Vector3d xyz_drake     = state_as_vector.segment<3>(0);
  const Vector4d quat_NB_drake = state_as_vector.segment<4>(3);
  const Vector3d w_NB_B_drake  = state_as_vector.segment<3>(7);
  const Vector3d v_NBo_B_drake = state_as_vector.segment<3>(10);

  const VectorXd stateDt_as_vector = stateDt_drake->CopyToVector();
  const Vector3d xyzDt_drake     = stateDt_as_vector.segment<3>(0);
  const Vector4d quatDt_NB_drake = stateDt_as_vector.segment<4>(3);
  const Vector3d wDt_NB_B_drake  = stateDt_as_vector.segment<3>(7);
  const Vector3d vDt_NBo_B_drake = stateDt_as_vector.segment<3>(10);

  // Reserve space for values returned by calculating exact solution.
  Quaterniond quatd_NB_exact;
  Vector4d quatDt_NB_exact;
  Vector3d w_NB_B_exact, wDt_NB_B_exact;
  Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;

  // Calculate exact analytical rotational solution.
  const double t = context.get_time();
  std::tie(quatd_NB_exact, quatDt_NB_exact, w_NB_B_exact, wDt_NB_B_exact) =
      CalculateExactRotationalSolutionNB(t, quat_NB_initial, w_NB_B_initial);

  // Get gravitational acceleration expressed in N (e.g. [0, 0, -9.8]).
  // Note: a_grav is an improperly-named public member of tree class (BAD).
  const RigidBodyTree<double>& tree = rigid_body_plant.get_rigid_body_tree();
  const Vector3d gravity = tree.a_grav.tail<3>();

  // Calculate exact analytical translational solution.
  // Exact analytical solution needs v_initial expressed in terms of Nx, Ny, Nz.
  const Eigen::Matrix3d R_NB_initial = quat_NB_initial.toRotationMatrix();
  const Vector3d v_NBo_N_initial = R_NB_initial * v_NBo_B_initial;
  std::tie(xyz_exact, xyzDt_exact, xyzDDt_exact) =
  CalculateExactTranslationalSolution(t, xyz_initial, v_NBo_N_initial, gravity);

  // Since more than one quaternion is associated with the same orientation,
  // compare Drake's canonical quaternion with exact canonical quaternion.
  const Quaterniond quatd_NB_drake = math::quat2eigenQuaternion(quat_NB_drake);
  const bool is_ok_quat = math::AreQuaternionsApproximatlyEqualInCanonicalForm(
                          quatd_NB_exact, quatd_NB_drake, tolerance);
  EXPECT_TRUE(is_ok_quat);

  // Convert quaternions to rotation matrix (also compare rotation matrices).
  Vector4d quat_NB_exact;
  quat_NB_exact << quatd_NB_exact.w(), quatd_NB_exact.x(),
                   quatd_NB_exact.y(), quatd_NB_exact.z();
  const Eigen::Matrix3d R_NB_drake = math::quat2rotmat(quat_NB_drake);
  const Eigen::Matrix3d R_NB_exact = math::quat2rotmat(quat_NB_exact);
  EXPECT_TRUE(R_NB_drake.isApprox(R_NB_exact, tolerance));

  // Drake: Compensate for definition of vDt = acceleration - w x v.
  const Vector3d w_cross_v_drake = w_NB_B_drake.cross(v_NBo_B_drake);
  const Vector3d xyzDDt_drake = R_NB_drake *(vDt_NBo_B_drake + w_cross_v_drake);

  // Exact: Compensate for definition of vDt = acceleration - w x v.
  const Eigen::Matrix3d R_BN_exact = R_NB_exact.inverse();
  const Vector3d v_NBo_B_exact = R_BN_exact * xyzDt_exact;
  const Vector3d w_cross_v_exact = w_NB_B_exact.cross(v_NBo_B_exact);
  const Vector3d vDt_NBo_B_exact = R_BN_exact * xyzDDt_exact - w_cross_v_exact;

  // Compare Drake and exact results.
  // TODO(mitiguy) Investigate why big factor (1600) needed to test wDt_NB_B.
  EXPECT_TRUE(CompareMatrices(w_NB_B_drake,       w_NB_B_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(wDt_NB_B_drake,  wDt_NB_B_exact, 1600*tolerance));
  EXPECT_TRUE(CompareMatrices(xyz_drake,             xyz_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDt_drake,         xyzDt_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDDt_drake,       xyzDDt_exact,  10*tolerance));
  EXPECT_TRUE(CompareMatrices(v_NBo_B_drake,     v_NBo_B_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(vDt_NBo_B_drake, vDt_NBo_B_exact,  10*tolerance));

  // Multi-step process to compare Drake vs exact time-derivative of quaternion.
  // Ensure time-derivative of Drake's quaternion satifies quarternionDt test.
  // Since more than one time-derivative of a quaternion is associated with the
  // same angular velocity, convert to angular velocity to compare results.
  EXPECT_TRUE(math::IsBothQuaternionAndQuaternionDtOK(quatd_NB_drake,
                                                quatDt_NB_drake, 16*tolerance));
  const Vector3d w_from_quatDt_drake =
      math::CalculateAngularVelocityExpressedInBFromQuaternionDt(
          quatd_NB_drake, quatDt_NB_drake);

  const Vector3d w_from_quatDt_exact =
      math::CalculateAngularVelocityExpressedInBFromQuaternionDt(
          quatd_NB_exact, quatDt_NB_exact);
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_drake, w_NB_B_drake, tolerance));
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_exact, w_NB_B_exact, tolerance));
}


/**
 * Numerically integrate rigid body plant's equations of motion from time t = 0
 * to time t_final with a 3rd-order variable-step Runge-Kutta integrator.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in,out] context
 *    On entry, context should be properly filled with all input to System
 *    (input ports, parameters, time = 0.0, state).  On output, context has been
 *    changed so its time = t_final and its state has been updated.
 * @param[in] dt_max
 *    The maximum (fixed) step size taken by the integrator.  If  t_final
 *    is not an exact multiple of dt, the final integration step is adjusted.
 * @param[in] t_final
 *    Value of time that signals the simulation to stop.
 * @param[in] maximum_absolute_error_per_integration_step
 *    Maximum allowable absolute error (for any variable being integrated) for
 *    the numerical-integrator's internal time-step (which can shrink or grow).
 */
void  IntegrateForwardWithVariableStepRungeKutta3(
                 const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
                 drake::systems::Context<double>* context,
                 const double dt_max, const double t_final,
                 const double maximum_absolute_error_per_integration_step) {
  DRAKE_DEMAND(context != NULL  &&  dt_max >= 0.0);

  // Integrate with variable-step Runge-Kutta3 integrator.
  systems::RungeKutta3Integrator<double> rk3(rigid_body_plant, context);
  rk3.set_maximum_step_size(dt_max);  // Need before Initialize (or exception).
  rk3.set_target_accuracy(maximum_absolute_error_per_integration_step);
  rk3.Initialize();

  // Integrate to within a small amount of dt of t_final.
  const double epsilon_based_on_dt_max = 1.0E-11 * dt_max;
  const double t_final_minus_epsilon = t_final - epsilon_based_on_dt_max;
  double t;
  while ( (t = context->get_time()) < t_final_minus_epsilon ) {
    const double dt = (t + dt_max > t_final) ? (t_final - t) : dt_max;
    rk3.StepOnceAtMost(dt, dt, dt);    // Step forward by at most dt.
  }
  DRAKE_DEMAND(std::abs(t-t_final) < epsilon_based_on_dt_max);
}


/** This function tests Drake's calculation of the time-derivative of the state
 * at many initial values. For certain initial values (somewhat randomly chosen)
 * this function also tests Drake's numerical integration of the rigid body's
 * equations of motion.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in,out] context
 *    On entry, context is properly sized for all input to System (input ports,
 *    parameters, time, state).  On output, context may be changed with
 *    specific values of time and state.
 * @param[out] stateDt_drake
 *    On output, stores time-derivatives of state.
 * @param[in] quat_NB_initial
 *   Initial value of the quaternion (which should already be normalized) that
 *   that relates Nx, Ny, Nz to Bx, By, Bz.
 *   Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
 * @param[in] w_NB_B_initial
 *   B's initial angular velocity in N, expressed in B.
 * @param[in] xyz_initial
 *   Initial values of x, y, z -- [the Nx, Ny, Nz measures of Bcm's (B's center
 *   of mass) position from a point No fixed in N (NewtonianFrame/World N)].
 * @param[in] v_NBo_B_initial
 *    Initial values of Bx, By, Bz measures of Bo's velocity in N, where Bo is
 *    Bcm.  Note: These values are not (in general) x', y', z' (i.e., these
 *    values are not time-derivatives of x, y,z).
 * @param[in] should_numerically_integrate
 *    True if also simulate x seconds and check accuracy of simulation results.
 */
void  TestDrakeSolutionForSpecificInitialValue(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    drake::systems::Context<double>* context,
    drake::systems::ContinuousState<double>* stateDt_drake,
    const Eigen::Quaterniond& quat_NB_initial,
    const Vector3d& w_NB_B_initial,
    const Vector3d& xyz_initial,
    const Vector3d& v_NBo_B_initial,
    const bool should_numerically_integrate) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL);

  // Get the state portion of the Context. State has unusual order/mearning.
  // State for joints::kQuaternion -- x,y,z, e0,e1,e2,e3, wx,wy,wz, vx,vy,vz.
  // Note: Bo is the origin of rigid cylinder B (here, coincident with Bcm).
  // Bx, By, Bz are fixed in B, with Bz along cylinder B's symmetric axis.
  // No is the origin of the Newtonian reference frame (World) N.
  // Nx, Ny, Nz are fixed in N, with Nz vertically upward (opposite gravity).
  // e0, e1, e2, e3 is the quaternion relating Nx, Ny, Nz to Bx, By, Bz,
  // with e0 being the scalar term in the quaternion.
  // w_NB_B is B's angular velocity in N expressed in B, [wx, wy, wz].
  // wDt_B is  B's angular acceleration in N, expressed in B, [wx', wy', wz'].
  // x, y, z are Bo's position from No, expressed in N.
  // v_NBo_B is Bo's velocity in N, expressed in B: [vx,vy,vz] not [x', y', z'].
  // vDt_B is the time-derivative in B of v_NBo_B, [vx', vy', vz'] - which is
  // not physically meaningful. Note: Bo's acceleration in N, expressed in B, is
  // calculated by [Kane, 1985, pg. 23], as: a_NBo_B = vDt_B + w_NB_B x v_NBo_B.
  //
  // - [Kane, 1985] "Dynamics: Theory and Applications," McGraw-Hill Book Co.,
  //   New York, 1985 (with D. A. Levinson).  Available for free .pdf download:
  //  https://ecommons.cornell.edu/handle/1813/637
  //
  // Concatenate these 4 Eigen column matrices into one Eigen column matrix.
  // Note: The state has a weird order (see previous comment).
  // Set state portion of Context from initial state.
  Eigen::Matrix<double, 13, 1> state_initial;
  state_initial << xyz_initial,
                   quat_NB_initial.w(),
                   quat_NB_initial.x(),
                   quat_NB_initial.y(),
                   quat_NB_initial.z(),
                   w_NB_B_initial,
                   v_NBo_B_initial;
  systems::VectorBase<double>& state_drake =
      *(context->get_mutable_continuous_state_vector());
  state_drake.SetFromVector(state_initial);

  // Ensure the time stored by context is set to 0.0 (initial value).
  context->set_time(0.0);

  // Test Drake's calculated values for time-derivative of state at time t = 0
  // versus exact analytical (closed-form) solution.
  // Initially, (time=0), Drake's results should be close to machine-precision.
  const double epsilon = std::numeric_limits<double>::epsilon();
  TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
                      rigid_body_plant, *context, stateDt_drake,
                      quat_NB_initial, w_NB_B_initial,
                      xyz_initial, v_NBo_B_initial, 50 * epsilon);

  // Maybe numerically integrate.
  if ( should_numerically_integrate ) {
    const double dt_max = 0.2, t_final = 10.0;
    const double maximum_absolute_error_per_integration_step = 1.0E-3;

    // Integrate forward, checking results frequently.
    IntegrateForwardWithVariableStepRungeKutta3(rigid_body_plant, context,
                 dt_max, t_final, maximum_absolute_error_per_integration_step);

    // After numerically integrating to t = t_final, test Drake's simulation
    // accuracy at t = t_final versus exact analytical (closed-form) solution.
    const double tolerance = maximum_absolute_error_per_integration_step;
    TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
                                     rigid_body_plant, *context, stateDt_drake,
                                     quat_NB_initial, w_NB_B_initial,
                                     xyz_initial, v_NBo_B_initial, tolerance);
  }
}


/** This function tests Drake's calculation of the time-derivative of the state
 * at many initial values. For certain initial values (somewhat randomly chosen)
 * this function also tests Drake's numerical integration of the rigid body's
 * equations of motion.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in,out] context
 *    On entry, context is properly sized for all input to System (input ports,
 *    parameters, time, state).  On output, context may be changed with
 *    specific values of time and state.
 * @param[out] stateDt_drake
 *    On output, stores time-derivatives of state.
 */
void  TestDrakeSolutionForVariousInitialValues(
      const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
      drake::systems::Context<double>* context,
      drake::systems::ContinuousState<double>* stateDt_drake) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL);

  // Create 3x1 matrix for wx, wy, wz (defined above).
  // Create 3x1 matrix for x, y, z (defined above).
  // Create 3x1 matrix for vx, vy, vz (defined above -- not x', y', z').
  const Vector3d w_NB_B_initial(2.0, 4.0, 6.0);
  const Vector3d xyz_initial(1.0, 2.0, 3.0);
  const Vector3d v_NBo_B_initial(-4.2, 5.5, 6.1);

  // Create 4x1 matrix for normalized quaternion e0, e1, e2, e3 (defined above).
  // Iterate through many initial values for quaternion.
  // Since cylinder B is axis-symmetric for axis Bz, iterate on BodyXY rotation
  // sequence with 0 <= thetaX <= 2*pi and 0 <= thetaY <= pi.
  unsigned int test_counter = 0u;
  for (double thetaX = 0; thetaX <= 2*M_PI; thetaX += 0.02*M_PI) {
    for (double thetaY = 0; thetaY <= M_PI; thetaY += 0.05*M_PI) {
      const Vector3<double> spaceXYZ_angles = {thetaX, thetaY, 0};
      const Vector4d drake_quat_NB_initial = math::rpy2quat(spaceXYZ_angles);
      Eigen::Quaterniond quat_NB_initial(drake_quat_NB_initial(0),
                                         drake_quat_NB_initial(1),
                                         drake_quat_NB_initial(2),
                                         drake_quat_NB_initial(3));

      // Since there are 100*20 = 2000 total tests of initial conditions, it is
      // time-consuming to numerically integrate for every initial condition.
      // Hence, numerical integration is only tested sporadically.
      const bool also_numerically_integrate = (test_counter++ == 120);
      TestDrakeSolutionForSpecificInitialValue(rigid_body_plant,
                                               context,
                                               stateDt_drake,
                                               quat_NB_initial,
                                               w_NB_B_initial,
                                               xyz_initial,
                                               v_NBo_B_initial,
                                               also_numerically_integrate);
    }
  }
}


/**
 * This function tests Drake's simulation of the motion of an axis-symmetric
 * rigid body B (uniform cylinder) in a Newtonian frame (World) N.  Since the
 * only external forces on B are uniform gravitational forces, there exists
 * an exact closed-form analytical solution for B's motion.  The closed-form
 * rotational solution is available since B is "torque-free", i.e., the moment
 * of all forces about B's mass center is zero.
 * Specifically, this function tests Drake's solution for quaternion, angular
 * velocity and angular acceleration expressed in B (body-frame).  This function
 * also tests Drake's solution for position, velocity, and acceleration
 * expressed in N (World).
 * Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
GTEST_TEST(uniformSolidCylinderTorqueFree, testA) {
  // Create path to .urdf file containing mass/inertia and geometry properties.
  const std::string urdf_name = "uniform_solid_cylinder.urdf";
  const std::string urdf_dir = "/multibody/benchmarks/"
      "cylinder_torque_free_analytical_solution/";
  const std::string urdf_dir_file_name = GetDrakePath() + urdf_dir + urdf_name;

  // Construct an empty RigidBodyTree.
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();

  // Populate RigidBodyTree tree by providing path to .urdf file,
  // second argument is enum FloatingBaseType that specifies the joint
  // connecting the robot's base link to ground/world. enum values are:
  // kFixed         welded to ground.
  // kRollPitchYaw  translates x,y,z and rotates 3D (by SpaceXYZ Euler angles).
  // kQuaternion    translates x,y,z and rotates 3D (by Euler parameters).
  const drake::multibody::joints::FloatingBaseType joint_type =
      drake::multibody::joints::kQuaternion;
  std::shared_ptr<RigidBodyFrame<double>> weld_to_frame = nullptr;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      urdf_dir_file_name, joint_type, weld_to_frame, tree.get());

  // Create a RigidBodyPlant which takes ownership of tree.
  drake::systems::RigidBodyPlant<double> rigid_body_plant(std::move(tree));

  // Create a Context which stores state and extra calculations.
  std::unique_ptr<systems::Context<double>> context =
      rigid_body_plant.CreateDefaultContext();

  // Allocate space to hold time-derivative of state_drake.
  std::unique_ptr<systems::ContinuousState<double>> stateDt_drake =
      rigid_body_plant.AllocateTimeDerivatives();

  TestDrakeSolutionForVariousInitialValues(rigid_body_plant, context.get(),
                                           stateDt_drake.get());
}



}  // namespace cylinder_torque_free_analytical_solution
}  // namespace benchmarks
}  // namespace drake
