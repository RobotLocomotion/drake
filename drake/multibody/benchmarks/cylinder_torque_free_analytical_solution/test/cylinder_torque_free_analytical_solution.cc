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
#include "drake/math/quaternion.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
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

/** Calculate quaternion's time-derivative from angular velocity and quaternion.
 * Algorithm from [Kane, 1983] Section 1.13, Pages 58-59.
 *
 * @param quat_AB Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat_AB is analogous to the rotation matrix R_AB.
 * @param w_AB_B  B's angular velocity in A, expressed in B.
 * @retval quatDt time-derivative of quat_AB, i.e., [e0', e1', e2', e3'].
 *
 * @note To avoid dependence on Eigen's internal ordering of elements in its
 * Quaternion class, herein we use `e0 = quat.w()', `e1 = quat.x()`, etc.
 * Return value `quatDt` *does* have a specific order as defined above.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (With P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
// TODO(mitiguy and Dai)  Create QuaternionDt class and update Doxygen.
template<typename T>
Vector4<T> CalculateQuaternionDtFromAngularVelocityExpressedInB(
    const Eigen::Quaternion<T>& quat_AB,  const Vector3<T>& w_AB_B ) {
  const T e0 = quat_AB.w(),  e1 = quat_AB.x(),
          e2 = quat_AB.y(),  e3 = quat_AB.z();
  const T wx = w_AB_B[0], wy = w_AB_B[1], wz = w_AB_B[2];

  const T e0Dt = 0.5*(-e1*wx - e2*wy - e3*wz);
  const T e1Dt = 0.5*(+e0*wx - e3*wy + e2*wz);
  const T e2Dt = 0.5*(+e3*wx + e0*wy - e1*wz);
  const T e3Dt = 0.5*(-e2*wx + e1*wy + e0*wz);

  return Vector4<T>(e0Dt, e1Dt, e2Dt, e3Dt);
}


/** Calculate how well the quaternion and quaternion's time derivative satisfy
 * the quaternion time-derivative constraint specified in [Kane, 1983]
 * Section 1.13, equations 12-13, page 59.  For a quaternion [e0, e1, e2, e3],
 * this algorithm uses the fact that:  e0^2 + e1^2 + e2^2 + e3^2 = 1,
 * whose time-derivative is 2*(e0*e0' + e1*e1' + e2*e2' + e3*e3') = 0.
 *
 * @param quat  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  time-derivative of `quat`, i.e., [e0', e1', e2', e3'].
 * @retval value of constraint - should be near 0 (may be positive or negative).
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
template <typename T>
T CalculateQuaternionDtConstraintFromQuaternionDt(
    const Eigen::Quaternion<T>& quat, const Vector4<T>& quatDt) {
  const T e0 = quat.w(), e1 = quat.x(), e2 = quat.y(), e3 = quat.z();
  const T e0Dt = quatDt[0], e1Dt = quatDt[1],
          e2Dt = quatDt[2], e3Dt = quatDt[3];

  return 2.0 * (e0*e0Dt + e1*e1Dt + e2*e2Dt + e3*e3Dt);
}


/** Tests how well the quaternion and quaternion's time derivative satisfy
 * the quaternion time-derivative constraint specified in [Kane, 1983]
 * Section 1.13, equations 12-13, page 59.  For a quaternion [e0, e1, e2, e3],
 * this algorithm uses the fact that:  e0^2 + e1^2 + e2^2 + e3^2 = 1,
 * whose time-derivative is 2*(e0*e0' + e1*e1' + e2*e2' + e3*e3') = 0.
 *
 * @param quat  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: A quaternion like quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  time-derivative of `quat`, i.e., [e0', e1', e2', e3'].
 * @returns true if both of the following constraints are satisfied:
 * a) e0^2 + e1^2 + e3^2 + e3^2 - 1 = 0,  within 800 * double precision epsilon.
 * b) 2*(e0*e0' + e1*e1' + e2*e2' + e3*e3') = 0   to within 800 * epsilon.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
template <typename T>
bool TestQuaternionDtConstraintFromQuaternionDt(
    const Eigen::Quaternion<T>& quat, const Vector4<T>& quatDt) {
  using std::abs;

  // For an accurate test, the quaternion should be reasonably accurate.
  const double double_epsilon = std::numeric_limits<double>::epsilon();
  const double tolerance = 800E15 * double_epsilon;
  const double quat_epsilon = abs(1.0 - quat.norm());
  const bool is_good_quat_norm = (quat_epsilon <= tolerance);
  if ( is_good_quat_norm == false ) return false;

  const double quatDt_test =
    CalculateQuaternionDtConstraintFromQuaternionDt(quat, quatDt);
  return (quatDt_test <= tolerance);
}

/** Calculate angular velocity from quaternion and quaternion's time derivative.
 * Algorithm from [Kane, 1983] Section 1.13, Pages 58-59.
 *
 * @param quat_AB  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., Ax, Ay, Az (A) to Bx, By, Bz (B).
 *   Note: quat_AB is analogous to the rotation matrix R_AB.
 * @param quatDt  time-derivative of `quat_AB`, i.e. [e0', e1', e2', e3'].
 * @retval w_AB_B  B's angular velocity in A, expressed in B.
 *
 * @note To avoid dependence on Eigen's internal ordering of elements in its
 * Quaternion class, herein we use `e0 = quat.w()', `e1 = quat.x()`, etc.
 * Parameter `quatDt` *does* have a specific order as defined above.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
// TODO(mitiguy and Dai)  Create QuaternionDt class and update Doxygen.
template <typename T>
Vector3<T> CalculateAngularVelocityExpressedInBFromQuaternionDt(
    const Eigen::Quaternion<T>& quat_AB, const Vector4<T>& quatDt) {
  const T e0 = quat_AB.w(), e1 = quat_AB.x(),
          e2 = quat_AB.y(), e3 = quat_AB.z();
  const T e0Dt = quatDt[0], e1Dt = quatDt[1],
          e2Dt = quatDt[2], e3Dt = quatDt[3];

#ifdef DRAKE_ASSERT_IS_ARMED
  const bool ok_arguments =
      TestQuaternionDtConstraintFromQuaternionDt(quat_AB, quatDt);
  DRAKE_ASSERT(ok_arguments);
#endif

  const T wx = 2*(-e1*e0Dt + e0*e1Dt + e3*e2Dt - e2*e3Dt);
  const T wy = 2*(-e2*e0Dt - e3*e1Dt + e0*e2Dt + e1*e3Dt);
  const T wz = 2*(-e3*e0Dt + e2*e1Dt - e1*e2Dt + e0*e3Dt);

  return Vector3<T>(wx, wy, wz);
}

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
  using std::sin;
  using std::cos;
  using std::sqrt;

  // Constant values of moments of inertia.
  const double I = 0.04;
  const double J = 0.02;

  // Initial values of wx, wy, wz.
  const double wx0 = w_NB_B_initial[0];
  const double wy0 = w_NB_B_initial[1];
  const double wz0 = w_NB_B_initial[2];

  // Intermediate calculations for quaternion solution.
  const double p = sqrt(wx0 * wx0 + wy0 * wy0 + (wz0 * J / I) * (wz0 * J / I));
  const double s = (I - J) / I * wz0;
  const double coef = wz0 * (J / (I * p));
  const double spt2 = sin(p * t / 2);
  const double cpt2 = cos(p * t / 2);
  const double sst2 = sin(s * t / 2);
  const double cst2 = cos(s * t / 2);

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
  const double wx =  wx0 * cos(s * t) + wy0 * sin(s * t);
  const double wy = -wx0 * sin(s * t) + wy0 * cos(s * t);
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
  const Vector4d quatDt = CalculateQuaternionDtFromAngularVelocityExpressedInB(
                          quat_NB, w_NB_B);

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


//-----------------------------------------------------------------------------
// TODO(Mitiguy) Remove function unless useful for more than Evan's PR# 4604.
// If more generally useful, create Doxygen for this function.
//-----------------------------------------------------------------------------
void  TestMapVelocityToQDot(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    const drake::systems::Context<double>& context,
    const Vector3d& w_NB_B_exact,
    const Vector3d& v_NBo_B_exact,
    const Vector4d& quatDt_NB_exact,
    const Vector3d& xyzDt_exact,
    const double tolerance) {

  // Form matrix of motion variables.
  Eigen::VectorXd motion_variables(6);
  motion_variables << w_NB_B_exact, v_NBo_B_exact;

  // Test whether MapVelocityToQDot accurately converts motion variables to
  // time-derivatives of coordinates.
  systems::BasicVector<double> coordinatesDt_from_map(7);
  rigid_body_plant.MapVelocityToQDot(context, motion_variables,
                                     &coordinatesDt_from_map);
  const double xDt_map = coordinatesDt_from_map[0];
  const double yDt_map = coordinatesDt_from_map[1];
  const double zDt_map = coordinatesDt_from_map[2];
  const double e0Dt_map = coordinatesDt_from_map[3];
  const double e1Dt_map = coordinatesDt_from_map[4];
  const double e2Dt_map = coordinatesDt_from_map[5];
  const double e3Dt_map = coordinatesDt_from_map[6];
  const Vector3d xyzDt_map(xDt_map, yDt_map, zDt_map);
  const Vector4d quatDt_map(e0Dt_map, e1Dt_map, e2Dt_map, e3Dt_map);
#if 0  // TODO(mitiguy) Remove these debug statements.
  std::cout << "\n\n--------------------------------------------------";
  std::cout << "\n----------- Test: MapVelocityToQDot --------------";
  std::cout << "\n--------------------------------------------------";
  std::cout << "\nquatDt_map =      \n" << quatDt_map;
  std::cout << "\nquatDt_NB_exact = \n" << quatDt_NB_exact;
  std::cout << "\n\nxyzDt_map=      \n" << xyzDt_map;
  std::cout << "\nxyzDt_exact =     \n" << xyzDt_exact;
  std::cout << "\n--------------------------------------------------\n";
#endif

  EXPECT_TRUE(CompareMatrices(xyzDt_map,      xyzDt_exact, tolerance));
  EXPECT_TRUE(CompareMatrices(quatDt_map, quatDt_NB_exact, tolerance));
}

//-----------------------------------------------------------------------------
// TODO(Mitiguy) Remove function unless useful for more than Evan's PR# 4604.
// If more generally useful, create Doxygen for this function.
//-----------------------------------------------------------------------------
void  TestMapQDotToVelocity(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    const drake::systems::Context<double>& context,
    const Vector4d& quatDt_NB_exact,
    const Vector3d& xyzDt_exact,
    const Vector3d& w_NB_B_exact,
    const Vector3d& v_NBo_B_exact,
    const double tolerance) {

  // Form matrix of time-derivative of coordinates.
  Eigen::VectorXd coordinatesDt(7);
  coordinatesDt << xyzDt_exact, quatDt_NB_exact;

  // Test whether MapQDotToVelocity accurately converts time-derivative of
  // coordinates to motion variables.
  systems::BasicVector<double> wv_from_map(6);
  rigid_body_plant.MapQDotToVelocity(context, coordinatesDt, &wv_from_map);
  const Vector3d w_map(wv_from_map[0], wv_from_map[1], wv_from_map[2]);
  const Vector3d v_map(wv_from_map[3], wv_from_map[4], wv_from_map[5]);
#if 0  // TODO(mitiguy) Remove these debug statements.
  std::cout << "\n\n--------------------------------------------------";
  std::cout << "\n----------- Test: MapQDotToVelocity --------------";
  std::cout << "\n--------------------------------------------------";
  std::cout << "\nw_map         = \n" << w_map;
  std::cout << "\nw_NB_B_exact  = \n" << w_NB_B_exact;
  std::cout << "\n\nv from map  = \n" << v_map;
  std::cout << "\nv_NBo_B_exact = \n" << v_NBo_B_exact;
  std::cout << "\n--------------------------------------------------\n";
#endif

  EXPECT_TRUE(CompareMatrices(w_map,  w_NB_B_exact, tolerance));
  EXPECT_TRUE(CompareMatrices(v_map, v_NBo_B_exact, tolerance));
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
  Vector4d quat_NB_exact, quatDt_NB_exact;
  Vector3d w_NB_B_exact, wDt_NB_B_exact;
  Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;

  // Calculate exact analytical rotational solution.
  const double t = context.get_time();
  Quaterniond quat_NB;
  std::tie(quat_NB, quatDt_NB_exact, w_NB_B_exact, wDt_NB_B_exact) =
      CalculateExactRotationalSolutionNB(t, quat_NB_initial, w_NB_B_initial);
  quat_NB_exact << quat_NB.w(), quat_NB.x(), quat_NB.y(), quat_NB.z();

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

  // Compare Drake quaternion with exact quaternion.
  // Since more than one quaternion is associated with the same orientation,
  // convert quaternions to rotation matrix and compare rotation matrices.
  // TODO(mitiguy) Add direct comparision of quaternions.
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

#if 0  // TODO(mitiguy) Remove these debug statements.
  std::cout << "\n\n quat_NB_drake\n"   << quat_NB_drake;
  std::cout << "\n quat_NB_exact\n"     << quat_NB_exact;
  std::cout << "\n\n quatDt_drake\n"    << quatDt_NB_drake;
  std::cout << "\n quatDt_NB_exact\n"   << quatDt_NB_exact;
  std::cout << "\n\n w_NB_B_drake\n"    << w_NB_B_drake;
  std::cout << "\n w_NB_B_exact\n"      << w_NB_B_exact;
  std::cout << "\n\n wDt_NB_B_drake\n"  << wDt_NB_B_drake;
  std::cout << "\n wDt_NB_B_exact\n"    << wDt_NB_B_exact;
  std::cout << "\n\n xyz_drake\n"       << xyz_drake;
  std::cout << "\n xyz_exact\n"         << xyz_exact;
  std::cout << "\n\n xyzDt_drake\n"     << xyzDt_drake;
  std::cout << "\n xyzDt_exact\n"       << xyzDt_exact;
  std::cout << "\n\n xyzDDt_drake\n"    << xyzDDt_drake;
  std::cout << "\n xyzDDt_exact\n"      << xyzDDt_exact;
  std::cout << "\n\n v_NBo_B_drake\n"   << v_NBo_B_drake;
  std::cout << "\n v_NBo_B_exact\n"     << v_NBo_B_exact;
  std::cout << "\n\n vDt_NBo_B_drake\n" << vDt_NBo_B_drake;
  std::cout << "\n vDt_NBo_B_exact\n"   << vDt_NBo_B_exact << "\n\n";
#endif

  // Compare Drake and exact results.
  // TODO(mitiguy) Investigate why big factor (1600) needed to test wDt_NB_B.
  EXPECT_TRUE(CompareMatrices(w_NB_B_drake,       w_NB_B_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(wDt_NB_B_drake,  wDt_NB_B_exact, 1600*tolerance));
  EXPECT_TRUE(CompareMatrices(xyz_drake,             xyz_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDt_drake,         xyzDt_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(xyzDDt_drake,       xyzDDt_exact,  10*tolerance));
  EXPECT_TRUE(CompareMatrices(v_NBo_B_drake,     v_NBo_B_exact,     tolerance));
  EXPECT_TRUE(CompareMatrices(vDt_NBo_B_drake, vDt_NBo_B_exact,  10*tolerance));

  // Two-step process to compare time-derivative of Drake quaternion with exact.
  // Since more than one time-derivative of a quaternion is associated with the
  // same angular velocity, convert to angular velocity to compare results.
  Quaterniond quaternion_drake = math::quat2eigenQuaternion(quat_NB_drake);
  const Vector3d w_from_quatDt_drake =
      CalculateAngularVelocityExpressedInBFromQuaternionDt(
          quaternion_drake, quatDt_NB_drake);
  Quaterniond quatd_exact = math::quat2eigenQuaternion(quat_NB_exact);
  const Vector3d w_from_quatDt_exact =
      CalculateAngularVelocityExpressedInBFromQuaternionDt(
          quatd_exact, quatDt_NB_exact);
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_drake, w_NB_B_drake, tolerance));
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_exact, w_NB_B_exact, tolerance));


  //--------------------------------------------------------------
  // EXTRA: Test MapQDotToVelocity and MapVelocityToQDot for Evan's PR #4604.
  //--------------------------------------------------------------
  TestMapVelocityToQDot(rigid_body_plant, context, w_NB_B_exact, v_NBo_B_exact,
                        quatDt_NB_exact, xyzDt_exact, tolerance);

  TestMapQDotToVelocity(rigid_body_plant, context, quatDt_NB_exact, xyzDt_exact,
                        w_NB_B_exact, v_NBo_B_exact, tolerance);
}


/**
 * Numerically integrate rigid body plant's equations of motion from time t = 0
 * to time t_final with a 2nd-order fixed-step Runge-Kutta integrator.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in,out] context
 *    On entry, context should be properly filled with all input to System
 *    (input ports, parameters, time = 0.0, state).  On output, context has been
 *    changed so its time = t_final and its state has been updated.
 * @param[in] dt
 *    The maximum (fixed) step size taken by the integrator.  If  t_final
 *    is not an exact multiple of dt, the final integration step is adjusted.
 * @param[in] t_final
 *    Value of time that signals the simulation to stop.
 */
void  IntegrateForwardWithFixedStepRungeKutta2(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    drake::systems::Context<double>* context,
    double dt, const double t_final) {
  DRAKE_DEMAND(context != NULL  &&  dt >= 0.0);

  // The initial value of time t is set from the context.
  double t = context->get_time();
  if ( t < t_final ) {
    // Integrate with fixed-sized steps with Runge-Kutta2 integrator.
    systems::RungeKutta2Integrator<double> rk2(rigid_body_plant, dt, context);
    rk2.Initialize();

    // Integrate to within a small amount of dt of t_final.
    const double epsilon_based_on_dt = 1.0E-9 * dt;
    const double t_final_minus_epsilon = t_final - epsilon_based_on_dt;
    do {
      if ( t + dt > t_final ) dt = t_final - t;
      rk2.StepOnceExactly(dt);           // Step forward by dt.
      t += dt;                           // or t = context->get_time();
    }while ( t < t_final_minus_epsilon );
  }
}


/**
 * This function tests Drake's calculation of time-derivative of the state at a
 * specific initial value, then numerically integrates the rigid body plant's
 * equations of motion from that initial value from time t=0 to t_final=1.0 sec.
 * with a 2nd-order fixed-step Runge-Kutta integrator.
 *
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
 */
void  TestDrakeSolutionForSpecificInitialValue(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    drake::systems::Context<double>* context,
    drake::systems::ContinuousState<double>* stateDt_drake,
    const Eigen::Quaterniond& quat_NB_initial,
    const Vector3d& w_NB_B_initial,
    const Vector3d& xyz_initial,
    const Vector3d& v_NBo_B_initial) {
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
  // TODO(mitiguy) Update comment/code when GitHub issue #4398 is fixed.
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

  // Numerically integrate with one of Drake's numerical integrators.
  const double dt = 1.0E-01, t_final = 1.0;
  IntegrateForwardWithFixedStepRungeKutta2(rigid_body_plant, context,
                                           dt, t_final);

  // After numerically integrating to t = t_final, test Drake's simulation
  // accuracy at t = t_final versus exact analytical (closed-form) solution.
  //--------------------------------------------------------------
  // TODO(Mitiguy and Drumwright) change tolerance to more reasonable value.
  //--------------------------------------------------------------
  TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
                      rigid_body_plant, *context, stateDt_drake,
                      quat_NB_initial, w_NB_B_initial,
                      xyz_initial, v_NBo_B_initial, 1.0E21 * epsilon);
}


/** Test Drake's calculation of the time-derivative of the state at various
 * initial values, then numerically integrate the rigid body plant's equations
 * of motion from that initial value from time t = 0 to t_final = 1.0 sec.
 * with a 2nd-order fixed-step Runge-Kutta integrator.
 *
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
  for (double thetaX = 0; thetaX <= 2* M_PI; thetaX += 0.01*M_PI) {
    for (double thetaY = 0; thetaY <= M_PI; thetaY += 0.2*M_PI) {
      const Vector3<double> spaceXYZ_angles = {thetaX, thetaY, 0};
      const Vector4d vector4d_NB_initial = math::rpy2quat(spaceXYZ_angles);
      Eigen::Quaterniond quat_NB_initial(vector4d_NB_initial(0),
                                         vector4d_NB_initial(1),
                                         vector4d_NB_initial(2),
                                         vector4d_NB_initial(3));

      TestDrakeSolutionForSpecificInitialValue(rigid_body_plant,
                                               context,
                                               stateDt_drake,
                                               quat_NB_initial,
                                               w_NB_B_initial,
                                               xyz_initial,
                                               v_NBo_B_initial);
    }
  }
}


/**
 * This function tests Drake solution for quaternion, angular velocity, and angular
 * acceleration expressed in body-frame, for torque-free rotational motion of an
 * axis-symmetric rigid body B (uniform cylinder) in Newtonian frame (world) N,
 * where torque-free means the moment of forces about B's mass center is zero.
 * Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 * @param uniformSolidCylinderTorqueFree Name of GTEST_TEST.
 * @param testA Name of sub-test (not relevant here).
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// Test Drake solution versus closed-form analytical solution.
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
