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
 * @retval quatDt  time-derivative of `quat`, ordered e0', e1', e2', e3'.
 *
 * @note Eigen's internal ordering for its Quaternion class should be
 * considered arbitrary. Herein we use `e0=quat.w()`, `e1=quat.x()`, etc.
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
    const Eigen::Quaternion<T>& quat,  const Vector3<T>& w_B ) {
  const T e0 = quat.w(), e1 = quat.x(), e2 = quat.y(), e3 = quat.z();
  const T wx = w_B[0], wy = w_B[1], wz = w_B[2];

  const T e0Dt = 0.5*(-e1*wx - e2*wy - e3*wz);
  const T e1Dt = 0.5*(+e0*wx - e3*wy + e2*wz);
  const T e2Dt = 0.5*(+e3*wx + e0*wy - e1*wz);
  const T e3Dt = 0.5*(-e2*wx + e1*wy + e0*wz);

  return Vector4<T>(e0Dt, e1Dt, e2Dt, e3Dt);
}


/** Calculate angular velocity from quaternion and quaternion's time derivative.
 * Algorithm from [Kane, 1983] Section 1.13, pages 58-59.
 *
 * @param quat  Quaternion e0, e1, e2, e3 that relates two right-handed
 *   orthogonal unitary bases e.g., ax, ay, az (A) to bx, by, bz (B).
 *   The quaternion `quat` easily converts to the rotation matrix R_AB
 * @param quatDt  time-derivative of `quat`, ordered e0', e1', e2', e3'.
 * @retval w_B  B's angular velocity in A, expressed in B (bx, by, bz).
 *
 * @note Eigen's internal ordering for its Quaternion class should be
 * considered arbitrary. Herein we use `e0=quat.w()`, `e1=quat.x()`, etc.
 * Parameter `quatDt` *does* have a specific order as defined above.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Move this and related methods (make unit test) to quaternion.h.
// TODO(mitiguy and Dai)  Create QuaternionDt class and update Doxygen.
template <typename T>
Vector3<T> CalculateAngularVelocityExpressedInBFromQuaternion(
    const Eigen::Quaternion<T>& quat, const Vector4<T>& quatDt) {
  const T e0 = quat.w(), e1 = quat.x(), e2 = quat.y(), e3 = quat.z();
  const T e0Dt = quatDt[0], e1Dt = quatDt[1],
          e2Dt = quatDt[2], e3Dt = quatDt[3];

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
 * @param w_initial Initial values of wx, wy, wz (which are the Bx, By, Bz
 * measures of B's angular velocity in N).  Note: Bx, By, Bz are right-handed
 * orthogonal unit vectors fixed in B, with Bz parallel to B's symmetry axis.
 * Note: An initial value of the quaternion is not passed to this algorithm as
 * this solution depends on it being [1, 0, 0, 0].  In other words, right-handed
 * orthogonal unit vectors N1, N2, N3 fixed in N are _chosen_ so they initially
 * align with Bx, By, Bz.  Yet another quaternion may be needed to account for
 * initial misalignment of N1, N1, N2 from a more physically-meaningfully set of
 * right-handed orthogonal unit vectors Nx, Ny, Nz fixed in N,
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat       | Vector4d quaternion representing B's orientation in N.
 * quatDt     | Vector4d with time-derivative of quaternion.
 * w          | Vector3d wx, wy, wz (B's angular velocity in N, expressed in B).
 * wDt        | Vector3d with time-derivative of wx, wy, wz.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
// TODO(mitiguy) Add a function to account for non-identity quat_initial.
std::tuple<Vector4d, Vector4d, Vector3d, Vector3d>
CalculateExactRotationalSolution(const double t, const Vector3d& w_initial) {
  using std::sin;
  using std::cos;
  using std::sqrt;

  // Constant values of moments of inertia.
  const double I = 0.04;
  const double J = 0.02;

  // Initial values of wx, wy, wz.
  const double wx0 = w_initial[0];
  const double wy0 = w_initial[1];
  const double wz0 = w_initial[2];

  // Intermediate calculations for quaternion solution.
  const double p = sqrt(wx0 * wx0 + wy0 * wy0 + (wz0 * J / I) * (wz0 * J / I));
  const double s = (I - J) / I * wz0;
  const double coef = wz0 * (J / (I * p));
  const double spt2 = sin(p * t / 2);
  const double cpt2 = cos(p * t / 2);
  const double sst2 = sin(s * t / 2);
  const double cst2 = cos(s * t / 2);

  // Analytical solution for quaternion (Euler parameters) e0(t), ... e3(t).
  const double e1 = spt2 / p * (wx0 * cst2 + wy0 * sst2);
  const double e2 = spt2 / p * (-wx0 * sst2 + wy0 * cst2);
  const double e3 =  coef * spt2 * cst2 + cpt2 * sst2;
  const double e0 = -coef * spt2 * sst2 + cpt2 * cst2;

  // Analytical solution for wx(t), wy(t), wz(t).
  const double wx =  wx0 * cos(s * t) + wy0 * sin(s * t);
  const double wy = -wx0 * sin(s * t) + wy0 * cos(s * t);
  const double wz =  wz0;

  // Analytical solution for time-derivative quaternion.
  const Vector4d eDt = CalculateQuaternionDtFromAngularVelocityExpressedInB(
      Quaterniond(e0, e1, e2, e3), Vector3d(wx, wy, wz));
  const double e0Dt = eDt[0];
  const double e1Dt = eDt[1];
  const double e2Dt = eDt[2];
  const double e3Dt = eDt[3];

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt =  (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;

  // Create a tuple to package for returning.
  std::tuple<Vector4d, Vector4d, Vector3d, Vector3d> returned_tuple;
  Vector4d& quat   = std::get<0>(returned_tuple);
  Vector4d& quatDt = std::get<1>(returned_tuple);
  Vector3d& w      = std::get<2>(returned_tuple);
  Vector3d& wDt    = std::get<3>(returned_tuple);

  // Fill returned_tuple with rotation results.
  quat   << e0, e1, e2, e3;
  quatDt << e0Dt, e1Dt, e2Dt, e3Dt;
  w      << wx, wy, wz;
  wDt    << wxDt, wyDt, wzDt;

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

  // Create a default RigidBodyTree constructor.
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

  // Create 4x1 matrix for quaternion e0, e1, e2, e3 (defined below).
  // Create 3x1 matrix for wx, wy, wz (defined below).
  // Create 3x1 matrix for x, y, z (defined below).
  // Create 3x1 matrix for vx, vy, vz (defined below -- not x', y', z').
  // TODO(mitiguy) Add a non-identity quat_initial.
  Vector4d quat_initial(1.0, 0.0, 0.0, 0.0);
  Vector3d w_initial(2.0, 4.0, 6.0);
  Vector3d xyz_initial(1.0, 2.0, 3.0);
  Vector3d v_initial(4.0, 5.0, 6.0);

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
  // State for joints::kQuaternion   -- x,y,z, e0,e1,e2,e3, wx,wy,wz, vx,vy,vz.
  // State for joints::kRollPitchYaw -- x,y,z, q1,q2,q3, x',y',z', q1',q2',q3'.
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
  // vDt_B is the time-derivative of v_B in B, [vx', vy', vz'] - not physically
  // meaningful. Note: Bo's acceleration in N, expressed in B, is calculated by
  // [Kane, 1985, pg. 23], as: a_NBo_B = vDt_B + w_B x v_B.
  //
  // - [Kane, 1985] "Dynamics: Theory and Applications," McGraw-Hill Book Co.,
  //   New York, 1985 (with D. A. Levinson).  Available for free .pdf download:
  //  https://ecommons.cornell.edu/handle/1813/637
  //
  // TODO(mitiguy) Update comment/code when GitHub issue #4398 is fixed.
  // TODO(mitiguy) kRollPitchYaw is documented here for my sanity/later use.
  // TODO(mitiguy) Confirm above comment about state kRollPitchYaw: x' or vx...?
  systems::VectorBase<double>& state_drake =
      *(context->get_mutable_continuous_state_vector());

  // Concatenate these 4 Eigen column matrices into one Eigen column matrix.
  // Note: The state has a weird order (see previous comment).
  Eigen::Matrix<double, 13, 1> state_initial;
  state_initial << xyz_initial, quat_initial, w_initial, v_initial;

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
  const Vector3d xyz_drake   = state_as_vector.segment<3>(0);
  const Vector4d quat_drake  = state_as_vector.segment<4>(3);
  const Vector3d w_drake     = state_as_vector.segment<3>(7);
  const Vector3d v_drake     = state_as_vector.segment<3>(10);

  const VectorXd stateDt_as_vector = stateDt_drake->CopyToVector();
  const Vector3d xyzDt_drake  = stateDt_as_vector.segment<3>(0);
  const Vector4d quatDt_drake = stateDt_as_vector.segment<4>(3);
  const Vector3d wDt_drake    = stateDt_as_vector.segment<3>(7);
  const Vector3d vDt_drake    = stateDt_as_vector.segment<3>(10);

  // Reserve space for values returned by calculating exact solution.
  Vector4d quat_exact, quatDt_exact;
  Vector3d w_exact, wDt_exact;
  Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;

  // Calculate exact analytical rotational solution.
  const double t = 0;
  std::tie(quat_exact, quatDt_exact, w_exact, wDt_exact) =
      CalculateExactRotationalSolution(t, w_initial);

  // Calculate exact analytical translational solution.
  std::tie(xyz_exact, xyzDt_exact, xyzDDt_exact) =
      CalculateExactTranslationalSolution(t, xyz_initial, v_initial, gravity);

#if 0  // TODO(mitiguy) Remove these debug statements.
  std::cout << "\n\n quat_drake\n"   << quat_drake;
  std::cout << "\n quat_exact\n"     << quat_exact;
  std::cout << "\n\n w_drake\n"      << w_drake;
  std::cout << "\n w_exact\n"        << w_exact;
  std::cout << "\n\n wDt_drake\n"    << wDt_drake;
  std::cout << "\n wDt_exact\n"      << wDt_exact;
  std::cout << "\n\n xyz_drake\n"    << xyz_drake;
  std::cout << "\n xyz_exact\n"      << xyz_exact;
  std::cout << "\n\n v_drake\n"      << v_drake;
  std::cout << "\n xyzDt_exact\n"    << xyzDt_exact;
  std::cout << "\n\n vDt_drake\n"    << vDt_drake;
  std::cout << "\n\n xyzDDt_exact\n" << xyzDDt_exact << "\n\n";
#endif

  // Compare Drake quaternion with exact quaternion.
  // Since more than one quaternion is associated with the same orientation,
  // convert quaternions to rotation matrix and compare rotation matrices.
  // Initially, (time=0), these matrices should be close to machine-precision,
  // which is approximately 2.22E-16.
  const double epsilon = Eigen::NumTraits<double>::epsilon();
  const Eigen::Matrix3d rotMatrix_drake = math::quat2rotmat(quat_drake);
  const Eigen::Matrix3d rotMatrix_exact = math::quat2rotmat(quat_exact);
  EXPECT_TRUE(rotMatrix_drake.isApprox(rotMatrix_exact, 20 * epsilon));

  // Two-step process to compare time-derivative of Drake quaternion with exact.
  // Since more than one time-derivative of a quaternion is associated with the
  // same angular velocity, convert to angular velocity to compare results.
  Quaterniond quaternion_drake = math::quat2eigenQuaternion(quat_drake);
  const Vector3d w_from_quatDt_drake =
      CalculateAngularVelocityExpressedInBFromQuaternion(quaternion_drake,
                                                         quatDt_drake);
  Quaterniond quatd_exact = math::quat2eigenQuaternion(quat_exact);
  const Vector3d w_from_quatDt_exact =
      CalculateAngularVelocityExpressedInBFromQuaternion(quatd_exact,
                                                         quatDt_exact);
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_drake, w_drake, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices(w_from_quatDt_exact, w_exact, 10 * epsilon));

  // Compare remaining drake and exact results.
  EXPECT_TRUE(CompareMatrices(w_drake,         w_exact, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices(wDt_drake,     wDt_exact, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices(xyz_drake,     xyz_exact, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices(xyzDt_drake, xyzDt_exact, 10 * epsilon));

  // Compensate for definition of vDt.
  // TODO(mitiguy) Premultiply by R_BN when non-identity quaternion.
  const Vector3d acceleration_difference = w_drake.cross(v_drake);
  const Vector3d vDt_test = xyzDDt_exact - acceleration_difference;
  EXPECT_TRUE(CompareMatrices(vDt_drake, vDt_test, 10 * epsilon));
}

}  // namespace cylinder_torque_free_analytical_solution
}  // namespace benchmarks
}  // namespace drake
