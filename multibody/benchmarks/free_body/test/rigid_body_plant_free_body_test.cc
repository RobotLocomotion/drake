// Purpose: Compare Drake with analytical closed-form solution from Section
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
#include "drake/multibody/benchmarks/free_body/free_body.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"

namespace drake {
namespace benchmarks {
namespace free_body {
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Quaterniond;


/**
 * This function tests Drake solution versus an exact solution for torque-free
 * motion of axis-symmetric uniform rigid cylinder B in Newtonian frame/World N,
 * where torque-free means the moment of forces about B's mass center is zero.
 * @param[in] rigid_body_plant A reference to the rigid-body being simulated.
 * @param[in] context All input to System (input ports, parameters, time, state)
 *   with cache for computations dependent on these values.
 * @param[in] torque_free_cylinder_solution Class that stores initial values,
 *   gravity, and methods to calculation the exact solution at time t.
 * @param[in] tolerance Minimum tolerance required to match results.
 * @param[out] stateDt_drake On output, stores derivative values at t = t_final.
 */
  void TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
       const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
       const drake::systems::Context<double>& context,
       const FreeBody& torque_free_cylinder_solution,
       const double tolerance,
       drake::systems::ContinuousState<double>* stateDt_drake) {
  DRAKE_DEMAND(stateDt_drake != NULL);

  // Pull the state at time t from the context.
  const systems::VectorBase<double>& state_drake =
      context.get_continuous_state_vector();

  // Calculate the time-derivatives of the state.
  rigid_body_plant.CalcTimeDerivatives(context, stateDt_drake);

  // Get the state and state time-derivatives for comparison.
  const VectorXd state_as_vector = state_drake.CopyToVector();
  const Vector3d xyz_drake      = state_as_vector.segment<3>(0);
  const Vector4d quat4_NB_drake = state_as_vector.segment<4>(3);
  const Vector3d w_NB_B_drake   = state_as_vector.segment<3>(7);
  const Vector3d v_NBo_B_drake  = state_as_vector.segment<3>(10);

  // xyzDt is [ẋ, ẏ, ż] Bo's velocity in N, expressed in N, i.e., the time-
  //           derivative of [x, y, z] (Bo's position from No, expressed in N).
  // quatDt_NB is the time-derivative of quat_NB, i.e., [ė0  ė1  ė2  ė3].
  // wDt_NB_B  is B's angular acceleration in N, expressed in B, and
  //           is equal to [ẇx, ẇy, ẇz], where [wx, wy, wz] is w_NB_B.
  // vDt_NBo_B is the time-derivative in B of v_NBo_B, [v̇x  v̇y  v̇z] -
  //           which is not physically meaningful.
  const VectorXd stateDt_as_vector = stateDt_drake->CopyToVector();
  const Vector3d xyzDt_drake     = stateDt_as_vector.segment<3>(0);
  const Vector4d quatDt_NB_drake = stateDt_as_vector.segment<4>(3);
  const Vector3d wDt_NB_B_drake  = stateDt_as_vector.segment<3>(7);
  const Vector3d vDt_NBo_B_drake = stateDt_as_vector.segment<3>(10);
  const Quaterniond quat_NB_drake = math::quat2eigenQuaternion(quat4_NB_drake);

  // Calculate exact analytical rotational solution at time t.
  const double t = context.get_time();
  Quaterniond quat_NB_exact;
  Vector4d quatDt_NB_exact;
  Vector3d w_NB_B_exact, wDt_NB_B_exact;
  std::tie(quat_NB_exact, quatDt_NB_exact, w_NB_B_exact, wDt_NB_B_exact) =
      torque_free_cylinder_solution.CalculateExactRotationalSolutionNB(t);

  // Calculate exact analytical translational solution at time t,
  // specifically, p_NoBcm_N, v_NBcm_N, a_NBcm_N.
  Vector3d xyz_exact, xyzDt_exact, xyzDDt_exact;
  std::tie(xyz_exact, xyzDt_exact, xyzDDt_exact) =
      torque_free_cylinder_solution.CalculateExactTranslationalSolution(t);

  // Since more than one quaternion is associated with the same orientation,
  // compare Drake's canonical quaternion with exact canonical quaternion.
  const bool is_ok_quat = math::AreQuaternionsEqualForOrientation(
                          quat_NB_exact, quat_NB_drake, tolerance);
  EXPECT_TRUE(is_ok_quat);

  // Convert quaternions to rotation matrix (also compare rotation matrices).
  const Eigen::Matrix3d R_NB_drake = quat_NB_drake.toRotationMatrix();
  const Eigen::Matrix3d R_NB_exact = quat_NB_exact.toRotationMatrix();
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
  EXPECT_TRUE(math::IsBothQuaternionAndQuaternionDtOK(
      quat_NB_drake, quatDt_NB_drake, 16*tolerance));

  EXPECT_TRUE(math::IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB(
      quat_NB_drake, quatDt_NB_drake, w_NB_B_drake, 16*tolerance));

  EXPECT_TRUE(math::IsQuaternionAndQuaternionDtEqualAngularVelocityExpressedInB(
      quat_NB_exact, quatDt_NB_exact, w_NB_B_exact, 16*tolerance));
}


/**
 * Numerically integrate rigid body plant's equations of motion from time t = 0
 * to time t_final with a 3rd-order variable-step Runge-Kutta integrator.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in] dt_max
 *    The maximum (fixed) step size taken by the integrator.  Note: t_final is
 *    not an exact multiple of dt, the final integration step is adjusted.
 * @param[in] t_final
 *    Value of time that signals the simulation to stop.
 *    t_final is not required to be an exact multiple of dt_max as the
 *    integration step is adjusted as necessary to land at exactly t_final.
 * @param[in] maximum_absolute_error_per_integration_step
 *    Maximum allowable absolute error (for any variable being integrated) for
 *    the numerical-integrator's internal time-step (which can shrink or grow).
 * @param[in] torque_free_cylinder_solution
 *   Data and methods calculate exact solution at time t.
 * @param[in,out] context
 *    On entry, context should be properly filled with all input to System
 *    (input ports, parameters, time = 0.0, state).  On output, context has been
 *    changed so its time = t_final and its state has been updated.
 * @param[out] stateDt_drake
 *   On output, stores time-derivatives of state.
 */
void  IntegrateForwardWithVariableStepRungeKutta3(
          const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
          const double dt_max, const double t_final,
          const double maximum_absolute_error_per_integration_step,
          const FreeBody& torque_free_cylinder_solution,
          drake::systems::Context<double>* context,
          drake::systems::ContinuousState<double>* stateDt_drake) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL  &&  dt_max >= 0.0);

  // Integrate with variable-step Runge-Kutta3 integrator.
  systems::RungeKutta3Integrator<double> rk3(rigid_body_plant, context);
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
    // accuracy versus exact analytical (closed-form) solution.
    const double tolerance = maximum_absolute_error_per_integration_step;
    TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
        rigid_body_plant, *context,
        torque_free_cylinder_solution, tolerance, stateDt_drake);

    const double t = context->get_time();
    if (t >= t_final_minus_epsilon) break;

    const double dt = (t + dt_max > t_final) ? (t_final - t) : dt_max;
    rk3.IntegrateAtMost(dt, dt, dt);    // Step forward by at most dt.
  }
}


/** This function tests Drake's calculation of the time-derivative of the state
 * at an initial condition, and depending on should_numerically_integrate, also
 * tests Drake's numerical integration of the rigid body's equations of motion.
 * @param[in] rigid_body_plant
 *    A reference to the rigid-body system being simulated.
 * @param[in] torque_free_cylinder_solution
 *    Data and methods calculate exact solution at time t.
 * @param[in] should_numerically_integrate
 *    True if also simulate x seconds and check accuracy of simulation results.
 * @param[in,out] context
 *    On entry, context is properly sized for all input to System (input ports,
 *    parameters, time, state).  On output, context may be changed with
 *    specific values of time and state.
 * @param[out] stateDt_drake
 *    On output, stores time-derivatives of state.
 */
void  TestDrakeSolutionForSpecificInitialValue(
    const drake::systems::RigidBodyPlant<double>& rigid_body_plant,
    const FreeBody& torque_free_cylinder_solution,
    const bool should_numerically_integrate,
    drake::systems::Context<double>* context,
    drake::systems::ContinuousState<double>* stateDt_drake) {
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL);

  // Set Drake's state (context) from initial values by concatenating
  // 4 smaller Eigen column matrices into one larger Eigen column matrix.
  //
  // Note: Drake's state has unusual order/mearning.  Drake's state for
  // joints::kQuaternion is: x,y,z, e0,e1,e2,e3, wx,wy,wz, vx,vy,vz.
  //
  // x, y, z are Bo's position from No (World origin), expressed in N, where Bo
  // is the origin of cylinder B. In this test, Bo is Bcm (B's center of mass).
  //
  // e0, e1, e2, e3 is the quaternion relating Nx, Ny, Nz to Bx, By, Bz,
  // with e0 being the scalar term in the quaternion and where
  // Nx, Ny, Nz are unit vectors fixed in N (World) and Bx, By, Bz are unit
  // vectors fixed in B, with Bz along cylinder B's symmetric axis.
  //
  // wx, wy, wz is w_NB_B, B's angular velocity in N expressed in B.
  //
  // v_NBo_B is Bo's velocity in N, expressed in B: [vx,vy,vz] not [ẋ, ẏ, ż].
  // vDt_B is the time-derivative in B of v_NBo_B, [v̇x, v̇y, v̇z] - which is
  // not physically meaningful. Note: Bo's acceleration in N, expressed in B, is
  // calculated by [Kane, 1985, pg. 23], as: a_NBo_B = vDt_B + w_NB_B x v_NBo_B.
  //
  // - [Kane, 1985] "Dynamics: Theory and Applications," McGraw-Hill Book Co.,
  //   New York, 1985 (with D. A. Levinson).  Available for free .pdf download:
  //  https://ecommons.cornell.edu/handle/1813/637
  const Quaterniond& quat_NB_initial = torque_free_cylinder_solution.
                                       get_quat_NB_initial();
  Eigen::Matrix<double, 13, 1> state_initial;
  state_initial << torque_free_cylinder_solution.get_p_NoBcm_N_initial(),
                   quat_NB_initial.w(),
                   quat_NB_initial.x(),
                   quat_NB_initial.y(),
                   quat_NB_initial.z(),
                   torque_free_cylinder_solution.get_w_NB_B_initial(),
                   torque_free_cylinder_solution.get_v_NBcm_B_initial();
  systems::VectorBase<double>& state_drake =
      context->get_mutable_continuous_state_vector();
  state_drake.SetFromVector(state_initial);

  // Ensure the time stored by context is set to 0.0 (initial value).
  context->set_time(0.0);

  // Test Drake's calculated values for time-derivative of state at time t = 0
  // versus exact analytical (closed-form) solution.
  // Initially (t = 0), Drake's results should be close to machine-precision.
  const double epsilon = std::numeric_limits<double>::epsilon();
  TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(
      rigid_body_plant, *context,
      torque_free_cylinder_solution, 50*epsilon, stateDt_drake);

  // Maybe numerically integrate.
  if (should_numerically_integrate) {
    const double dt_max = 0.2, t_final = 10.0;
    const double maximum_absolute_error_per_integration_step = 1.0E-3;

    // Integrate forward, testing Drake's results vs. exact solution frequently.
    IntegrateForwardWithVariableStepRungeKutta3(
        rigid_body_plant, dt_max,
        t_final, maximum_absolute_error_per_integration_step,
        torque_free_cylinder_solution, context, stateDt_drake);
  }
}


/** This function sets up various initial values for Drake to test against an
 * exact solution. For one (mostly arbitrary) initial value, this also tests
 * Drake's numerical integration of the rigid body's equations of motion.
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
  DRAKE_DEMAND(context != nullptr  &&  stateDt_drake != nullptr);

  // Store initial values in a class that can calculate an exact solution.
  // Store gravitational acceleration expressed in N (e.g. [0, 0, -9.8]).
  // Note: a_grav is improper public member of RigidBodyTree class (BAD).
  Quaterniond quat_NB_initial(1, 0, 0, 0);
  const Vector3d w_NB_B_initial(2.0, 4.0, 6.0);
  const Vector3d p_NoBcm_N_initial(1.0, 2.0, 3.0);
  const Vector3d v_NBcm_B_initial(-4.2, 5.5, 6.1);
  const RigidBodyTree<double>& tree = rigid_body_plant.get_rigid_body_tree();
  const Vector3d gravity = tree.a_grav.tail<3>();
  FreeBody torque_free_cylinder_exact(
      quat_NB_initial, w_NB_B_initial,
      p_NoBcm_N_initial, v_NBcm_B_initial, gravity);

  // Test a variety of initial normalized quaternions.
  // Since cylinder B is axis-symmetric for axis Bz, iterate on BodyXY rotation
  // sequence with 0 <= thetaX <= 2*pi and 0 <= thetaY <= pi.
  int test_counter = 0;
  for (double thetaX = 0; thetaX <= 2*M_PI; thetaX += 0.02*M_PI) {
    for (double thetaY = 0; thetaY <= M_PI; thetaY += 0.05*M_PI) {
      const Vector3<double> spaceXYZ_angles = {thetaX, thetaY, 0};
      const Vector4d drake_quat_NB_initial = math::rpy2quat(spaceXYZ_angles);
      quat_NB_initial = Quaterniond(drake_quat_NB_initial(0),
                                    drake_quat_NB_initial(1),
                                    drake_quat_NB_initial(2),
                                    drake_quat_NB_initial(3));
      torque_free_cylinder_exact.set_quat_NB_initial(quat_NB_initial);

      // Since there are 100*20 = 2000 total tests of initial conditions, only
      // perform time-consuming numerically integration infrequently.
      const bool also_numerically_integrate = (test_counter++ == 120);
      TestDrakeSolutionForSpecificInitialValue(rigid_body_plant,
                                               torque_free_cylinder_exact,
                                               also_numerically_integrate,
                                               context, stateDt_drake);
    }
  }
}


/**
 * This function tests Drake's simulation of the motion of an axis-symmetric
 * rigid body B (uniform cylinder) in a Newtonian frame (World) N.  Since the
 * only external forces on B are uniform gravitational forces, the moment of
 * all forces about B's mass center is zero ("torque-free"), there exists an
 * exact closed-form analytical solution for B's motion.  Specifically, this
 * function tests Drake's solution against an exact solution for: quaternion,
 * angular velocity, and angular acceleration expressed in B (body-frame);
 * position, velocity, and acceleration expressed in N (World).
 * Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
GTEST_TEST(uniformSolidCylinderTorqueFree, testA) {
  // Create path to .urdf file containing mass/inertia and geometry properties.
  const std::string urdf_name = "uniform_solid_cylinder.urdf";
  const std::string urdf_dir = "drake/multibody/benchmarks/"
      "free_body/";
  const std::string urdf_dir_file_name = FindResourceOrThrow(
      urdf_dir + urdf_name);

  // Construct an empty RigidBodyTree.
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();

  // Populate RigidBodyTree tree by providing path to .urdf file,
  // second argument is enum FloatingBaseType that specifies the joint
  // connecting the robot's base link to ground/world. Possibilities are:
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

}  // namespace free_body
}  // namespace benchmarks
}  // namespace drake
