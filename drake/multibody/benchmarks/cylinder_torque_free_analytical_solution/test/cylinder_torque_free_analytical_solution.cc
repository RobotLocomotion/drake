// Purpose: Compare Drake with analytical closed-form solution from Section
//          1.13 (pgs. 60-62) of Spacecraft Dynamics, by Kane & Levinson.
//          This closed-form solution includes both angular velocity and
//          Euler parameters for an axis-symmetric rigid body B, when the moment
//          of forces on B about Bcm (B's center of mass) is zero (torque-free).
//-----------------------------------------------------------------------------
#include <tuple>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"


namespace drake {
namespace benchmarks {
namespace cylinder_torque_free_analytical_solution {
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

/** Calculate angular velocity from quaternion and quaternion's time derivative.
 * @param quat    quaternion e0, e1, e2, e3 that relates two right-handed
 * orthogonal unitary bases e.g., ax, ay, az to bx, by, bz.
 * @param quatDt  time-derivative of quaternion quat, i.e., e0', e1', e2', e3'.
 * @returns Vector3d with B's angular velocity in A, expressed in bx, by, bz.
 */
// TODO(mitiguy) Move later to /drake/math/someFileName.m.
Vector3d CalculateAngularVelocityFromQuaternion( const Vector4d& quat,
                                                 const Vector4d& quatDt ) {
  const double e0 = quat[0];
  const double e1 = quat[1];
  const double e2 = quat[2];
  const double e3 = quat[3];
  const double e0Dt = quatDt[0];
  const double e1Dt = quatDt[1];
  const double e2Dt = quatDt[2];
  const double e3Dt = quatDt[3];

  const double wx = 2*(-e1 * e0Dt  +  e0 * e1Dt  +  e3 * e2Dt  -  e2 * e3Dt);
  const double wy = 2*(-e2 * e0Dt  -  e3 * e1Dt  +  e0 * e2Dt  +  e1 * e3Dt);
  const double wz = 2*(-e3 * e0Dt  +  e2 * e1Dt  -  e1 * e2Dt  +  e0 * e3Dt);

  return Vector3d(wx, wy, wz);
}


/**
 * Calculates exact solutions for Euler parameters, angular velocity measures,
 * and their time derivatives for torque-free rotational motion of an axis-
 * symmetric rigid body B (e.g., uniform solid cylinder) in Newtonian frame N.
 * @param t Current value of time.
 * @param w_initial Initial values of wx, wy, wz (which are the Bx, By, Bz
 * measures of B's angular velocity in N).  Note: Bx, By, Bz are right-handed
 * orthogonal unit vectors fixed in B, with Bz parallel to B's symmetry axis.
 * @param xyz_initial Initial values of x, y, z [which are the Nx, Ny, Nz
 * measures of Bcm's position from No (a point fixed in N)].  Note: Nx, Ny, Nz
 * are right-handed orthogonal unit vectors fixed in N.
 * @param xyzDt_initial Initial values of x', y', z' (which are time-derivatives
 * of x, y,z -- and equal to the Nx, Ny, Nz measures of Bcm's velocity in N).
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat       | Eigen::Vector4d quaternion representing B's orientation in N.
 * quatDt     | Eigen::Vector4d with time-derivative of quaternion.
 * w          | Eigen::Vector3d with wx, wy, wz.
 * wDt        | Eigen::Vector3d with time-derivative of wx, wy, wz.
 * xyz        | Eigen::Vector3d with x, y, z.
 * xyzDt      | Eigen::Vector3d with x', y', z'.
 * xyzDDt     | Eigen::Vector3D with x'', y'', z''.
 */
std::tuple<Vector4d, Vector4d, Vector3d, Vector3d, Vector3d, Vector3d, Vector3d>
    CalculateExactSolution(const double t,
                           const Vector3d& w_initial,
                           const Vector3d& xyz_initial,
                           const Vector3d& xyzDt_initial) {
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
  const double e0Dt = -0.5 * e1 * wx - 0.5 * e2 * wy - 0.5 * e3 * wz;
  const double e1Dt =  0.5 * e0 * wx + 0.5 * e2 * wz - 0.5 * e3 * wy;
  const double e2Dt =  0.5 * e0 * wy + 0.5 * e3 * wx - 0.5 * e1 * wz;
  const double e3Dt =  0.5 * e0 * wz + 0.5 * e1 * wy - 0.5 * e2 * wx;

  // Analytical solution for time-derivatives of wx, wy, wz.
  const double wxDt =  (1 - J / I) * wy * wz;
  const double wyDt = (-1 + J / I) * wx * wz;
  const double wzDt = 0.0;

  // Initial values of x, y, z and x', y', z'.
  const double x0 = xyz_initial[0];
  const double y0 = xyz_initial[1];
  const double z0 = xyz_initial[2];
  const double xDt0 = xyzDt_initial[0];
  const double yDt0 = xyzDt_initial[1];
  const double zDt0 = xyzDt_initial[2];

  // Analytical solution for x'', y'', z'' (no forces on body).
  const double x_acceleration = 0.0;
  const double y_acceleration = 0.0;
  const double z_acceleration = -9.81;

  // Analytical solution for x', y', z' (constant acceleration).
  const double xDt = xDt0 + x_acceleration * t;
  const double yDt = yDt0 + y_acceleration * t;
  const double zDt = zDt0 + z_acceleration * t;

  // Analytical solution for x, y, z (constant acceleration).
  const double x = x0 + xDt0*t + 0.5 * x_acceleration * t;
  const double y = y0 + yDt0*t + 0.5 * y_acceleration * t;
  const double z = z0 + zDt0*t + 0.5 * z_acceleration * t;

  // Create a tuple to package for returning.
  std::tuple<Vector4d, Vector4d, Vector3d, Vector3d,
             Vector3d, Vector3d, Vector3d> returned_tuple;
  Vector4d& quat   = std::get<0>(returned_tuple);
  Vector4d& quatDt = std::get<1>(returned_tuple);
  Vector3d& w      = std::get<2>(returned_tuple);
  Vector3d& wDt    = std::get<3>(returned_tuple);
  Vector3d& xyz    = std::get<4>(returned_tuple);
  Vector3d& xyzDt  = std::get<5>(returned_tuple);
  Vector3d& xyzDDt = std::get<6>(returned_tuple);

  // Fill returned_tuple with rotation results.
  quat << e0, e1, e2, e3;
  quatDt << e0Dt, e1Dt, e2Dt, e3Dt;
  w << wx, wy, wz;
  wDt << wxDt, wyDt, wzDt;
  xyz << x, y, z;
  xyzDt << xDt, yDt, zDt;
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
  const std::string urdf_dirFilename = GetDrakePath() + urdf_dir + urdf_name;

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
  std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr;
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      urdf_dirFilename, joint_type, weld_to_frame, tree.get());

  // Create a RigidBodyPlant which takes ownership of tree.
  // Note: unique_ptr helps ensure tree is only managed/deleted once.
  drake::systems::RigidBodyPlant<double> rigid_body_plant(std::move(tree));

  // Create a Context: Which holds state and extra calculations.
  std::unique_ptr<systems::Context<double>> context =
      rigid_body_plant.CreateDefaultContext();

  // Get the state portion of the Context. State has weird/inconsistent order.
  // State for joints::kQuaternion   -- x,y,z, e0,e1,e2,e3, wx,wy,wz, x',y',z'.
  // State for joints::kRollPitchYaw -- x,y,z, q1,q2,q3, x',y',z', q1',q2',q3'
  // (where q1=Roll, q2=Pitch, q3=Yaw for SpaceXYZ rotation sequence).
  // TODO(mitiguy) Update comment/code when GitHub issue #4398 is fixed.
  // TODO(mitiguy) kRollPitchYaw is documented here for my sanity/later use.
  systems::VectorBase<double> &state_drake =
      *(context->get_mutable_continuous_state_vector());

  // Create 3x1 Eigen column matrix for x, y, z.
  // Create 3x1 Eigen column matrix for x', y', z'.
  Vector3d xyz_initial(1.0, 2.0, 3.0);
  Vector3d xyzDt_initial(4.0, 5.0, 6.0);

  // Create 4x1 Eigen column matrix for quaternion e0, e1, e2, e3.
  // Create 3x1 Eigen column matrix for wx, wy, wz.
  // TODO(mitiguy) Add a non-identity quat_initial.
  Vector4d quat_initial(1.0, 0.0, 0.0, 0.0);
  Vector3d w_initial(2.0, 4.0, 6.0);

  // Concatenate these 4 Eigen column matrices into one Eigen column matrix.
  // Note: The state has a weird order (see previous comment).
  Eigen::Matrix<double, 13, 1> eigen_state;
  eigen_state << xyz_initial, quat_initial, w_initial, xyzDt_initial;

  // Set state portion of Context using eigen_state.
  state_drake.SetFromVector(eigen_state);

  // Construct enough space to hold time-derivative of state_drake.
  std::unique_ptr<systems::ContinuousState<double>> ds =
      rigid_body_plant.AllocateTimeDerivatives();
  systems::ContinuousState<double> *stateDt_drake = ds.get();

  // Setup an empty input port - that serves no purpose but to frustrate me.
  const int num_actuators = rigid_body_plant.get_num_actuators();
  std::unique_ptr<systems::FreestandingInputPort> port =
      std::make_unique<systems::FreestandingInputPort>(
          std::make_unique<systems::BasicVector<double>>(num_actuators));
  context->SetInputPort(0, std::move(port));

  // Evaluate the time-derivatives of the state.
  systems::Context<double> *raw_context = context.get();
  rigid_body_plant.EvalTimeDerivatives(*raw_context, stateDt_drake);

  // TODO(mitiguy) Add simulation and check numerical integrator.

  // Get the state and state time-derivatives for comparison.
  // State for joints::kQuaternion -- x,y,z, e0,e1,e2,e3, wx,wy,wz, x',y',z'.
  // TODO(mitiguy) Update comment/code when GitHub issue #4398 is fixed.
  VectorXd state_eigen = state_drake.CopyToVector();
  Vector3d xyz_eigen   = state_eigen.segment<3>(0);
  Vector4d quat_eigen  = state_eigen.segment<4>(3);
  Vector3d w_eigen     = state_eigen.segment<3>(7);
  Vector3d xyzDt_eigen = state_eigen.segment<3>(10);

  // TODO(mitiguy) Uncomment xyzDDt_eigen when GitHub issue #4398 is fixed.
  VectorXd stateDt_eigen = stateDt_drake->CopyToVector();
  Vector4d quatDt_eigen = stateDt_eigen.segment<4>(3);
  Vector3d wDt_eigen    = stateDt_eigen.segment<3>(7);
  //Vector3d xyzDDt_eigen = stateDt_eigen.segment<3>(10);

  // Reserve space for values returned by calculating exact solution.
  Vector4d quat_exact, quatDt_exact;
  Vector3d w_exact, wDt_exact;
  Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;

  // Calculate exact analytical solution.
  const double t = 0;
  std::tie(quat_exact, quatDt_exact, w_exact, wDt_exact,
           xyz_exact, xyzDt_exact, xyzDDt_exact) =
      CalculateExactSolution(t, w_initial, xyz_initial, xyzDt_initial);

  // Compare Drake quaternion with exact quaternion.
  // Since more than one quaternion is associated with the same orientation,
  // convert quaternions to rotation matrix and compare rotation matrices.
  // Initially, (time=0), these matrices should be close to machine-precision,
  // which is approximately 2.22E-16.
  const double epsilon = Eigen::NumTraits<double>::epsilon();
  const Eigen::Matrix3d rotMatrix_eigen = math::quat2rotmat(quat_eigen);
  const Eigen::Matrix3d rotMatrix_exact = math::quat2rotmat(quat_exact);
  EXPECT_TRUE(rotMatrix_eigen.isApprox(rotMatrix_exact, 20 * epsilon));

  // Two-step process to compare time-derivative of Drake quaternion with exact.
  // Since more than one time-derivative of a quaternion is associated with the
  // same angular velocity, convert to angular velocity to compare results.
  const Vector3d w_from_quat_quatDt_eigen =
      CalculateAngularVelocityFromQuaternion( quat_eigen, quatDt_eigen);
  const Vector3d w_from_quat_quatDt_exact =
      CalculateAngularVelocityFromQuaternion( quat_exact, quatDt_exact);
  EXPECT_TRUE(CompareMatrices(w_from_quat_quatDt_eigen, w_eigen, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices(w_from_quat_quatDt_exact, w_exact, 10 * epsilon));

  // Compare remaining drake and exact results.
  EXPECT_TRUE(CompareMatrices(     w_eigen,      w_exact, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices(   wDt_eigen,    wDt_exact, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices(   xyz_eigen,    xyz_exact, 10 * epsilon));
  EXPECT_TRUE(CompareMatrices( xyzDt_eigen,  xyzDt_exact, 10 * epsilon));
  //EXPECT_TRUE(CompareMatrices(xyzDDt_eigen, xyzDDt_exact, 1.0E+15));
  // TODO(mitiguy) Uncomment last test when GitHub issue #4398 is fixed.
}

}  // namespace cylinder_torque_free_analytical_solution
}  // namespace benchmarks
}  // namespace drake
