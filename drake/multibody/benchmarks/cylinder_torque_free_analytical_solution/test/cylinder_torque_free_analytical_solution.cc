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
 * Algorithm from [Kane, 1983] Section 1.13, pages 58-59.
 *
 * @param quat Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., ax, ay, az (A) to bx, by, bz (B).
 *   The quaternion `quat` easily converts to the rotation matrix R_AB.
 * @param w_B  B's angular velocity in A, expressed in B (bx, by, bz).
 * @retval quatDt_B  time-derivative in B of `quat`, i.e., [e0', e1', e2', e3'].
 *
 * @note Eigen's internal ordering for its Quaternion class should be
 * considered arbitrary. Herein we use `e0=quat.w()`, `e1=quat.x()`, etc.
 * Return value `quatDt_B` *does* have a specific order as defined above.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (With P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
// TODO(mitiguy and Dai)  Create QuaternionDt class and update Doxygen.
template<typename T>
Vector4<T> CalculateQuaternionDtInBFromAngularVelocityExpressedInB(
    const Eigen::Quaternion<T>& quat,  const Vector3<T>& w_B ) {
  const T e0 = quat.w(), e1 = quat.x(), e2 = quat.y(), e3 = quat.z();
  const T wx = w_B[0], wy = w_B[1], wz = w_B[2];

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
 *   orthogonal unitary bases e.g., ax, ay, az (A) to bx, by, bz (B).
 * @param quatDt_B  time-derivative in B of `quat`, i.e. [e0', e1', e2', e3'].
 * @retval value of constraint - may be positive or negative, but near 0.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
template <typename T>
double   CalculateQuaternionDtConstraintFromQuaternionDtExpressedInB(
    const Eigen::Quaternion<T>& quat, const Vector4<T>& quatDt_B) {
  const T e0 = quat.w(), e1 = quat.x(), e2 = quat.y(), e3 = quat.z();
  const T e0Dt = quatDt_B[0], e1Dt = quatDt_B[1],
          e2Dt = quatDt_B[2], e3Dt = quatDt_B[3];

  return 2.0 * (e0*e0Dt + e1*e1Dt + e2*e2Dt + e3*e3Dt);
}


/** Tests how well the quaternion and quaternion's time derivative satisfy
 * the quaternion time-derivative constraint specified in [Kane, 1983]
 * Section 1.13, equations 12-13, page 59.  For a quaternion [e0, e1, e2, e3],
 * this algorithm uses the fact that:  e0^2 + e1^2 + e2^2 + e3^2 = 1,
 * whose time-derivative is 2*(e0*e0' + e1*e1' + e2*e2' + e3*e3') = 0.
 *
 * @param quat  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., ax, ay, az (A) to bx, by, bz (B).
 * @param quatDt_B  time-derivative in B of `quat`, i.e. [e0', e1', e2', e3'].
 * @retval value of constraint - may be positive or negative, but near 0.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
template <typename T>
bool TestQuaternionDtConstraintFromQuaternionDtExpressedInB(
    const Eigen::Quaternion<T>& quat, const Vector4<T>& quatDt_B) {

  // For an accurate test, the quaternion should be reasonably accurate.
  const double double_epsilon = Eigen::NumTraits<double>::epsilon();
  const double quat_epsilon = abs(1.0 - quat.norm());
  const bool is_quat_larger = quat_epsilon > double_epsilon;
  const double epsilon = is_quat_larger ? quat_epsilon : double_epsilon;
  const double quat_normDt =
    CalculateQuaternionDtConstraintFromQuaternionDtExpressedInB(quat, quatDt_B);

  const bool is_good_quat_norm   = (quat_epsilon <= 800 * double_epsilon);
  const bool is_good_quat_normDt = (quat_normDt  <= 100 * epsilon);
  return is_good_quat_norm && is_good_quat_normDt;
}

/** Calculate angular velocity from quaternion and quaternion's time derivative.
 * Algorithm from [Kane, 1983] Section 1.13, pages 58-59.
 *
 * @param quat  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., ax, ay, az (A) to bx, by, bz (B).
 *   The quaternion `quat` easily converts to the rotation matrix R_AB
 * @param quatDt_B  time-derivative in B of `quat`, i.e. [e0', e1', e2', e3'].
 * @retval w_B  B's angular velocity in A, expressed in B (bx, by, bz).
 *
 * @note Eigen's internal ordering for its Quaternion class should be
 * considered arbitrary. Herein we use `e0=quat.w()`, `e1=quat.x()`, etc.
 * Parameter `quatDt_B` *does* have a specific order as defined above.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
// TODO(mitiguy and Dai)  Create QuaternionDt class and update Doxygen.
template <typename T>
Vector3<T> CalculateAngularVelocityExpressedInBFromQuaternionDtExpressedInB(
    const Eigen::Quaternion<T>& quat, const Vector4<T>& quatDt_B) {
  const T e0 = quat.w(), e1 = quat.x(), e2 = quat.y(), e3 = quat.z();
  const T e0Dt = quatDt_B[0], e1Dt = quatDt_B[1],
          e2Dt = quatDt_B[2], e3Dt = quatDt_B[3];

#ifdef DRAKE_ASSERT_IS_ARMED
  const bool ok_arguments =
      TestQuaternionDtConstraintFromQuaternionDtExpressedInB(quat, quatDt_B);
  DRAKE_ASSERT(ok_arguments);
#endif

  const T wx = 2*(-e1*e0Dt + e0*e1Dt + e3*e2Dt - e2*e3Dt);
  const T wy = 2*(-e2*e0Dt - e3*e1Dt + e0*e2Dt + e1*e3Dt);
  const T wz = 2*(-e3*e0Dt + e2*e1Dt - e1*e2Dt + e0*e3Dt);

  return Vector3<T>(wx, wy, wz);
}

/**
 * Calculates exact solutions for quaternion and angular velocity expressed in
 * body-frame, and their time derivatives for torque-free rotational motion of
 * axis-symmetric rigid body B (uniform cylinder) in Newtonian frame (world) N.
 * Algorithm from [Kane, 1983] Sections 1.13 and 3.1, pages 60-62 and 159-169.
 * @param t Current value of time.
 * @param quat_initial Initial value of the quaternion that characterizes the
 *    R_NB rotation matrix that relates two sets of right-handed orthogonal unit
 *    vectors, with set Nx, Ny, Nz fixed in N and set Bx, By, Bz fixed in B.
 *    Note: Bz is parallel to B's symmetry axis.
 * @param w_initial_B  B's initial angular velocity in N, expressed in B.
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat       | Quaternion representing B's orientation in N: [e0, e1, e2, e3].
 * quatDt_B   | Time-derivative in B of quaternion, i.e., [e0', e1', e2', e3'].
 * w_B        | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
 * wDt_B      | Time-derivative in B of w, i.e., [wx', wy', wz'].
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
std::tuple<Vector4d, Vector4d, Vector3d, Vector3d>
CalculateExactRotationalSolution(const double t,
                                 const Vector4d& quat_initial,
                                 const Vector3d& w_initial_B) {
  using std::sin;
  using std::cos;
  using std::sqrt;

  // Constant values of moments of inertia.
  const double I = 0.04;
  const double J = 0.02;

  // Initial values of wx, wy, wz.
  const double wx0 = w_initial_B[0];
  const double wy0 = w_initial_B[1];
  const double wz0 = w_initial_B[2];

  // Intermediate calculations for quaternion solution.
  const double p = sqrt(wx0 * wx0 + wy0 * wy0 + (wz0 * J / I) * (wz0 * J / I));
  const double s = (I - J) / I * wz0;
  const double coef = wz0 * (J / (I * p));
  const double spt2 = sin(p * t / 2);
  const double cpt2 = cos(p * t / 2);
  const double sst2 = sin(s * t / 2);
  const double cst2 = cos(s * t / 2);

  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is another set Ax, Ay, Az of right-handed orthogonal unit vectors
  // which are fixed in N (Newtonian frame) but initially aligned to Bx, By, Bz.
  // Kane's analytical solution for quat_AB [eB0, eB1, eB2, eB3] characterizes
  // the R_AB rotation matrix relating Ax, Ay, Az to Bx, By, Bz.
  // (not the quaternion relating A to N)
  const double eB1 = spt2 / p * (wx0 * cst2 + wy0 * sst2);
  const double eB2 = spt2 / p * (-wx0 * sst2 + wy0 * cst2);
  const double eB3 =  coef * spt2 * cst2 + cpt2 * sst2;
  const double eB0 = -coef * spt2 * sst2 + cpt2 * cst2;
  const Eigen::Quaterniond quat_AB(eB0, eB1, eB2, eB3);

  // Multiply (Hamilton product) the quaternions characterizing the rotation
  // matrices R_NA and R_AB to produce the quaternion characterizing R_NB.
  // In other words, account for q_initial (which is quat_NA) to calculate the
  // quaternion quat_NB that charaterizes the R_NB rotation matrix.
  const double eA0 = quat_initial[0], eA1 = quat_initial[1],
               eA2 = quat_initial[2], eA3 = quat_initial[3];
  const Eigen::Quaterniond quat_NA(eA0, eA1, eA2, eA3);
  const Eigen::Quaterniond quat_NB = quat_NA * quat_AB;

  // Analytical solution for wx(t), wy(t), wz(t).
  const double wx =  wx0 * cos(s * t) + wy0 * sin(s * t);
  const double wy = -wx0 * sin(s * t) + wy0 * cos(s * t);
  const double wz =  wz0;

  // Analytical solution for time-derivative quaternion in B.
  const Vector4d eDt = CalculateQuaternionDtInBFromAngularVelocityExpressedInB(
                       quat_NB, Vector3d(wx, wy, wz) );

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt =  (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;

  // Create a tuple to package for returning.
  std::tuple<Vector4d, Vector4d, Vector3d, Vector3d> returned_tuple;
  Vector4d& quat     = std::get<0>(returned_tuple);
  Vector4d& quatDt_B = std::get<1>(returned_tuple);
  Vector3d& w_B      = std::get<2>(returned_tuple);
  Vector3d& wDt_B    = std::get<3>(returned_tuple);

  // Fill returned_tuple with rotation results.
  quat     << quat_NB.w(), quat_NB.x(), quat_NB.y(), quat_NB.z();
  quatDt_B << eDt;
  w_B      << wx, wy, wz;
  wDt_B    << wxDt, wyDt, wzDt;

  return returned_tuple;
}

/**
 * Calculates exact solutions for translational motion of an arbitrary rigid
 * body B in a Newtonian frame (world) N.  Algorithm from high-school physics.
 * @param t Current value of time.
 * @param xyz_initial Initial values of x, y, z -- [the Nx, Ny, Nz measures of
 * Bcm's (B's center of mass) position from a point No fixed in world N].
 * Note: Nx, Ny, Nz are right-handed orthogonal unit vectors fixed in N.
 * @param xyzDt_initial Initial values of x', y', z' (which are time-derivatives
 * of x, y,z -- and equal to the Nx, Ny, Nz measures of Bcm's velocity in N).
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
// Test Drake solution versus closed-form analytical solution.
// GTEST_TEST: 1st arg is name of test, 2nd arg is name of sub-test.
GTEST_TEST(uniformSolidCylinderTorqueFree, testA) {
  // Create path to .urdf file containing mass/inertia and geometry properties.
  const std::string urdf_name = "uniform_solid_cylinder.urdf";
  const std::string urdf_dir = "/multibody/benchmarks/"
      "cylinder_torque_free_analytical_solution/";
  const std::string urdf_dir_file_name = GetDrakePath() + urdf_dir + urdf_name;

  // Loop over a number of initial orientations.
  for (double theta_initial = 0; theta_initial <= M_PI;
       theta_initial += M_PI/10.0) {
    // Create a default RigidBodyTree constructor.
    std::unique_ptr<RigidBodyTree<double>> tree =
        std::make_unique<RigidBodyTree<double>>();

    // Populate RigidBodyTree tree by providing path to .urdf file,
    // second argument is enum FloatingBaseType that specifies the joint
    // connecting the robot's base link to ground/world. enum values are:
    // kFixed         welded to ground.
    // kRollPitchYaw  translates x,y,z and rotates 3D (by SpaceXYZ Euler
    //                angles).
    // kQuaternion    translates x,y,z and rotates 3D (by Euler parameters).
    const drake::multibody::joints::FloatingBaseType joint_type =
        drake::multibody::joints::kQuaternion;
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame = nullptr;
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        urdf_dir_file_name, joint_type, weld_to_frame, tree.get());

    // Create 4x1 matrix for quaternion e0, e1, e2, e3 (defined below).
    // Create 3x1 matrix for wx, wy, wz (defined below).
    // Create 3x1 matrix for x, y, z (defined below).
    // Create 3x1 matrix for vx, vy, vz (defined below -- not x', y', z').
    const double e0_initial = cos(theta_initial / 2);
    const double e1_initial = sin(theta_initial / 2) * 1.0;
    const double e2_initial = sin(theta_initial / 2) * 0.0;
    const double e3_initial = sin(theta_initial / 2) * 0.0;
    Vector4d quat_initial(e0_initial, e1_initial, e2_initial, e3_initial);
    Vector3d w_initial_B(2.0, 4.0, 6.0);
    Vector3d xyz_initial(1.0, 2.0, 3.0);
    Vector3d v_initial_B(4.0, 5.0, 6.0);

    // Query drake for gravitational acceleration before moving tree.
    // Note: a_grav is an improperly-named public member of tree class (BAD).
    const Vector3d gravity = tree->a_grav.tail<3>();

    // Create a RigidBodyPlant which takes ownership of tree.
    // Note: unique_ptr helps ensure tree is only managed/deleted once.
    drake::systems::RigidBodyPlant<double> rigid_body_plant(std::move(tree));

    // Create a Context which stores state and extra calculations.
    std::unique_ptr<systems::Context<double>> context =
        rigid_body_plant.CreateDefaultContext();

    // Get the state portion of the Context. State has weird/inconsistent order.
    // State for joints::kQuaternion   -- x,y,z, e0,e1,e2,e3, wx,wy,wz, vx,vy,vz
    // State for joints::kRollPitchYaw -- x,y,z, q1,q2,q3, x',y',z', q1',q2',q3'
    // (where q1=Roll, q2=Pitch, q3=Yaw for SpaceXYZ rotation sequence).
    // Note: Bo is the origin of rigid cylinder B (here, coincident with Bcm).
    // Bx, By, Bz are fixed in B, with Bz along cylinder B's symmetric axis.
    // No is the origin of the Newtonian reference frame N (world).
    // Nx, Ny, Nz are fixed in N, with Nz vertically upward (opposite gravity).
    // e0, e1, e2, e3 is the quaternion relating Nx, Ny, Nz to Bx, By, Bz,
    // with e0 being the scalar term in the quaternion.
    // w_B is B's angular velocity in N expressed in B, [wx, wy, wz].
    // wDt_B is  B's angular acceleration in N, expressed in B, [wx', wy', wz'].
    // x, y, z are Bo's position from No, expressed in N.
    // v_B is Bo's velocity in N, expressed in B, [vx, vy, vz], not [x', y', z']
    // vDt_B is time-derivative of v_B in B, [vx', vy', vz'] - not physically
    // meaningful. Note: Bo's acceleration in N, expressed in B, is calculated
    // by [Kane, 1985, pg. 23], as: a_NBo_B = vDt_B + w_B x v_B.
    //
    // - [Kane, 1985] "Dynamics: Theory and Applications," McGraw-Hill Book Co.,
    //   New York, 1985 (with D. A. Levinson).  Available for free .pdf download
    //   at https://ecommons.cornell.edu/handle/1813/637
    //
    // TODO(mitiguy) Update comment/code when GitHub issue #4398 is fixed.
    // TODO(mitiguy) kRollPitchYaw is documented here for my sanity/later use.
    // TODO(mitiguy) Confirm above comment about state kRollPitchYaw: x' or
    //               vx...?
    systems::VectorBase<double> &state_drake =
        *(context->get_mutable_continuous_state_vector());

    // Concatenate these 4 Eigen column matrices into one Eigen column matrix.
    // Note: The state has a weird order (see previous comment).
    Eigen::Matrix<double, 13, 1> state_initial;
    state_initial << xyz_initial, quat_initial, w_initial_B, v_initial_B;

    // Set state portion of Context from initial state.
    state_drake.SetFromVector(state_initial);

    // Allocate space to hold time-derivative of state_drake.
    std::unique_ptr<systems::ContinuousState<double>> stateDt_drake =
        rigid_body_plant.AllocateTimeDerivatives();

    // Even though there are no actuators here we have to create a zero-length
    // actuator input port.
    const int num_actuators = rigid_body_plant.get_num_actuators();
    context->FixInputPort(0, VectorXd::Zero(num_actuators));

    // Evaluate the time-derivatives of the state.
    rigid_body_plant.CalcTimeDerivatives(*context, stateDt_drake.get());

    // TODO(mitiguy) Add simulation and check numerical integrator.

    // Get the state (defined above) and state time-derivatives for comparison.
    const VectorXd state_as_vector = state_drake.CopyToVector();
    const Vector3d xyz_drake = state_as_vector.segment<3>(0);
    const Vector4d quat_drake = state_as_vector.segment<4>(3);
    const Vector3d w_drake = state_as_vector.segment<3>(7);
    const Vector3d v_drake = state_as_vector.segment<3>(10);

    const VectorXd stateDt_as_vector = stateDt_drake->CopyToVector();
    const Vector3d xyzDt_drake = stateDt_as_vector.segment<3>(0);
    const Vector4d quatDt_drake = stateDt_as_vector.segment<4>(3);
    const Vector3d wDt_drake = stateDt_as_vector.segment<3>(7);
    const Vector3d vDt_drake = stateDt_as_vector.segment<3>(10);

    // Reserve space for values returned by calculating exact solution.
    Vector4d quat_exact, quatDt_exact;
    Vector3d w_exact, wDt_exact;
    Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;

    // Calculate exact analytical rotational solution.
    const double t = 0;
    std::tie(quat_exact, quatDt_exact, w_exact, wDt_exact) =
        CalculateExactRotationalSolution(t, quat_initial, w_initial_B);

    // Calculate exact analytical translational solution.
    // Exact analytical solution needs v_initial expressed in terms of Nx, Ny,
    // Nz.
    const Eigen::Matrix3d R_NB_initial = math::quat2rotmat(quat_initial);
    const Vector3d v_initial_N = R_NB_initial * v_initial_B;
    std::tie(xyz_exact, xyzDt_exact, xyzDDt_exact) =
        CalculateExactTranslationalSolution(t,
                                            xyz_initial,
                                            v_initial_N,
                                            gravity);

    // Compare Drake quaternion with exact quaternion.
    // Since more than one quaternion is associated with the same orientation,
    // convert quaternions to rotation matrix and compare rotation matrices.
    // Initially, (time=0), these matrices should be close to machine-precision,
    // which is approximately 2.22E-16.
    const double epsilon = Eigen::NumTraits<double>::epsilon();
    const Eigen::Matrix3d R_NB_drake = math::quat2rotmat(quat_drake);
    const Eigen::Matrix3d R_NB_exact = math::quat2rotmat(quat_exact);
    EXPECT_TRUE(R_NB_drake.isApprox(R_NB_exact, 50 * epsilon));

    // Drake: Compensate for definition of vDt = acceleration - w x v.
    const Vector3d w_cross_v_drake = w_drake.cross(v_drake);
    const Vector3d xyzDDt_drake = R_NB_drake * (vDt_drake + w_cross_v_drake);

    // Exact: Compensate for definition of vDt = acceleration - w x v.
    const Eigen::Matrix3d R_BN_exact = R_NB_exact.inverse();
    const Vector3d v_exact = R_BN_exact * xyzDt_exact;
    const Vector3d w_cross_v_exact = w_exact.cross(v_exact);
    const Vector3d vDt_exact = R_BN_exact * xyzDDt_exact - w_cross_v_exact;

#if 1  // TODO(mitiguy) Remove these debug statements.
    std::cout << "\n\n quat_drake\n" << quat_drake;
    std::cout << "\n quat_exact\n" << quat_exact;
    std::cout << "\n\n quatDt_drake\n" << quatDt_drake;
    std::cout << "\n quatDt_exact\n" << quatDt_exact;
    std::cout << "\n\n w_drake\n" << w_drake;
    std::cout << "\n w_exact\n" << w_exact;
    std::cout << "\n\n wDt_drake\n" << wDt_drake;
    std::cout << "\n wDt_exact\n" << wDt_exact;
    std::cout << "\n\n xyz_drake\n" << xyz_drake;
    std::cout << "\n xyz_exact\n" << xyz_exact;
    std::cout << "\n\n xyzDt_drake\n" << xyzDt_drake;
    std::cout << "\n xyzDt_exact\n" << xyzDt_exact;
    std::cout << "\n\n xyzDDt_drake\n" << xyzDDt_drake;
    std::cout << "\n xyzDDt_exact\n" << xyzDDt_exact;
    std::cout << "\n\n v_drake\n" << v_drake;
    std::cout << "\n v_exact\n" << v_exact;
    std::cout << "\n\n vDt_drake\n" << vDt_drake;
    std::cout << "\n vDt_exact\n" << vDt_exact << "\n\n";
#endif

    // Compare remaining drake and exact results.
    EXPECT_TRUE(CompareMatrices(w_drake, w_exact, 50 * epsilon));
    EXPECT_TRUE(CompareMatrices(wDt_drake, wDt_exact, 10000 * epsilon));
    EXPECT_TRUE(CompareMatrices(xyz_drake, xyz_exact, 50 * epsilon));
    EXPECT_TRUE(CompareMatrices(xyzDt_drake, xyzDt_exact, 50 * epsilon));
    EXPECT_TRUE(CompareMatrices(xyzDDt_drake, xyzDDt_exact, 100 * epsilon));
    EXPECT_TRUE(CompareMatrices(v_drake, v_exact, 50 * epsilon));
    EXPECT_TRUE(CompareMatrices(vDt_drake, vDt_exact, 100 * epsilon));

    // Two-step process to compare time-derivative of Drake quaternion with
    // exact. Since more than one time-derivative of a quaternion is associated
    // with the same angular velocity, convert to angular velocity to compare
    // results.
    Quaterniond quaternion_drake = math::quat2eigenQuaternion(quat_drake);
    const Vector3d w_from_quatDt_drake =
        CalculateAngularVelocityExpressedInBFromQuaternionDtExpressedInB(
            quaternion_drake, quatDt_drake);
    Quaterniond quatd_exact = math::quat2eigenQuaternion(quat_exact);
    const Vector3d w_from_quatDt_exact =
        CalculateAngularVelocityExpressedInBFromQuaternionDtExpressedInB(
            quatd_exact, quatDt_exact);
    EXPECT_TRUE(CompareMatrices(w_from_quatDt_drake, w_drake, 50 * epsilon));
    EXPECT_TRUE(CompareMatrices(w_from_quatDt_exact, w_exact, 50 * epsilon));

    //--------------------------------------------------------------
    // EXTRA: Test MapQDotToVelocity and MapVelocityToQDot for Evan.
    // TODO(Mitiguy,Drumwright) lose BadFix.
    //--------------------------------------------------------------
    const double BadFix = 1.0E16;  // 1.0E16;  // 1.0E-5;
    #define Test_MapQDotToVelocity_1_or_MapVelocityToQDotFalse_0    0
    #if Test_MapQDotToVelocity_1_or_MapVelocityToQDotFalse_0
    // Form matrix of time-derivative of coordinates.
    Eigen::VectorXd coordinatesDt(7);
    coordinatesDt << xyzDt_drake, quatDt_drake;

    // Test if MapQDotToVelocity accurately converts time-derivative of
    // coordinates to motion variables.
    systems::BasicVector<double> wv_from_map(6);
    rigid_body_plant.MapQDotToVelocity(*context, coordinatesDt, &wv_from_map);
    const Vector3d w_map(wv_from_map[0], wv_from_map[1], wv_from_map[2]);
    const Vector3d v_map(wv_from_map[3], wv_from_map[4], wv_from_map[5]);
    std::cout << "\n\n--------------------------------------------------";
    std::cout << "\n---------- TestA: MapQDotToVelocity --------------";
    std::cout << "\n--------------------------------------------------";
    std::cout << "\nw from map = \n" << w_map;
    std::cout << "\nw exact = \n" << w_exact;
    std::cout << "\n\nv from map = \n" << v_map;
    std::cout << "\nv accurate (drake) = \n" << v_drake;
    std::cout << "\n--------------------------------------------------\n";
    EXPECT_TRUE(CompareMatrices(w_map, w_exact, BadFix*10 * epsilon));
    EXPECT_TRUE(CompareMatrices(v_map, v_drake, BadFix*10 * epsilon));
    #else

    // Form matrix of motion variables.
    Eigen::VectorXd motion_variables(6);
    motion_variables << w_drake, v_drake;

    // Test that MapVelocityToQDot accurately converts motion variables to
    // time-derivatives of coordinates.
    systems::BasicVector<double> coordinatesDt_from_map(7);
    rigid_body_plant.MapVelocityToQDot(*context, motion_variables,
                                       &coordinatesDt_from_map);
    const double xDt_map = coordinatesDt_from_map[0];
    const double yDt_map = coordinatesDt_from_map[1];
    const double zDt_map = coordinatesDt_from_map[2];
    const double q0Dt_map = coordinatesDt_from_map[3];
    const double q1Dt_map = coordinatesDt_from_map[4];
    const double q2Dt_map = coordinatesDt_from_map[5];
    const double q3Dt_map = coordinatesDt_from_map[6];
    const Vector3d xyzDt_map(xDt_map, yDt_map, zDt_map);
    const Vector4d quatDt_map(q0Dt_map, q1Dt_map, q2Dt_map, q3Dt_map);
    std::cout << "\n\n--------------------------------------------------";
    std::cout << "\n---------- TestB: MapVelocityToQDot --------------";
    std::cout << "\n--------------------------------------------------";
    std::cout << "\nxyzDt from map = \n" << xyzDt_map;
    std::cout << "\nxyzDt exact = \n" << xyzDt_exact;
    std::cout << "\n\nquatDt from map = \n" << quatDt_map;
    std::cout << "\nquatDt exact = \n" << quatDt_exact;
    std::cout << "\n--------------------------------------------------\n";
    EXPECT_TRUE(CompareMatrices(xyzDt_map, xyzDt_exact, BadFix * 10 * epsilon));
    EXPECT_TRUE(CompareMatrices(quatDt_map,
                                quatDt_exact,
                                BadFix * 10 * epsilon));
    #endif
  }
}

}  // namespace cylinder_torque_free_analytical_solution
}  // namespace benchmarks
}  // namespace drake
