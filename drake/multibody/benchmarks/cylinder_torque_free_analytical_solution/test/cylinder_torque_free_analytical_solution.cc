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
#include "drake/math/autodiff.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
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

//-----------------------------------------------------------------------------
// The purpose of the TorqueFreeCylinderExactSolution class is to provide the
// data (initial values and gravity) and methods for calculating the exact
// analytical solution for the translational and rotational motion of an axis-
// symmetric rigid body B (uniform cylinder) in a Newtonian frame (World) N.
// Since the only external forces on B are uniform gravitational forces, there
// exists an exact closed-form analytical solution for B's motion. The closed-
// form rotational solution is available since B is "torque-free", i.e., the
// moment of all forces about B's mass center is zero.
// This class calculates cylinder B's quaternion, angular velocity and angular
// acceleration expressed in B (body-frame) as well as the position, velocity,
// acceleration of Bcm (B's center of mass) in N (World).
// Algorithm from [Kane, 1983] Sections 1.13 and 3.1, Pages 60-62 and 159-169.
//
// - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
//   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
//   https://ecommons.cornell.edu/handle/1813/637
//------------------------------------------------------------------------------
class TorqueFreeCylinderExactSolution{
 public:
  // Constructors and destructor.
  TorqueFreeCylinderExactSolution() = delete;
  TorqueFreeCylinderExactSolution(const Quaterniond& quat_NB_initial,
                                  const Vector3d& w_NB_B_initial,
                                  const Vector3d& p_NoBcm_N_initial,
                                  const Vector3d& v_NBcm_B_initial,
                                  const Vector3d& gravity_N) {
    set_quat_NB_initial(quat_NB_initial);
    set_w_NB_B_initial(w_NB_B_initial);
    set_p_NoBcm_N_initial(p_NoBcm_N_initial);
    set_v_NBcm_B_initial(v_NBcm_B_initial);
    SetUniformGravityExpressedInWorld(gravity_N);
  }
  ~TorqueFreeCylinderExactSolution() {}

  // Get methods for initial values and gravity.
  const Quaterniond&  get_quat_NB_initial() const { return quat_NB_initial_; }
  const Vector3d&  get_w_NB_B_initial() const { return w_NB_B_initial_; }
  const Vector3d&  get_p_NoBcm_N_initial() const { return p_NoBcm_N_initial_; }
  const Vector3d&  get_v_NBcm_B_initial()  const { return v_NBcm_B_initial_; }
  const Vector3d&  get_uniform_gravity_expressed_in_world() const
                         { return uniform_gravity_expressed_in_world_; }
  const Vector3d  GetInitialVelocityOfBcmInWorldExpressedInWorld() const {
      const Eigen::Matrix3d R_NB_initial = quat_NB_initial_.toRotationMatrix();
      return R_NB_initial * v_NBcm_B_initial_;
  }

  // Set methods for initial values and gravity.
  void  set_quat_NB_initial(const Quaterniond& quat_NB_initial)
        { quat_NB_initial_ = quat_NB_initial; }
  void  set_w_NB_B_initial(const Vector3d& w_NB_B_initial)
        { w_NB_B_initial_ = w_NB_B_initial; }
  void  set_p_NoBcm_N_initial(const Vector3d& p_NoBcm_N_initial)
        { p_NoBcm_N_initial_ = p_NoBcm_N_initial; }
  void  set_v_NBcm_B_initial(const Vector3d& v_NBcm_B_initial)
        { v_NBcm_B_initial_ = v_NBcm_B_initial; }
  void  SetUniformGravityExpressedInWorld(const Vector3d& gravity)
        { uniform_gravity_expressed_in_world_ = gravity; }

  // This method calculates quat_NB, quatDt, w_NB_B, alpha_NB_B at time t.
  std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d>
  CalculateExactRotationalSolutionNB(const double t) const;

  // This method calculates p_NoBcm_N, v_NBcm_N, a_NBcm_N, at time t.
  std::tuple<Vector3d, Vector3d, Vector3d>
  CalculateExactTranslationalSolution(const double t) const;

 private:
  // This "helper" method calculates quat_AB, w_NB_B, and alpha_NB_B at time t.
  std::tuple<Quaterniond, Vector3d, Vector3d>
  CalculateExactRotationalSolutionABInitiallyAligned(const double t) const;

  // quat_NB_initial_ is the initial (t=0) value of the quaternion that relates
  // unit vectors Nx, Ny, Nz fixed in World N (e.g., Nz vertically upward) to
  // unit vectors Bx, By, Bz fixed in cylinder B (Bz parallel to symmetry axis)
  // Note: The quaternion should already be normalized before it is set.
  // Note: quat_NB_initial is analogous to the initial rotation matrix R_NB.
  Quaterniond quat_NB_initial_;

  // w_NB_B_initial_  is B's initial angular velocity in N, expressed in B.
  Vector3d w_NB_B_initial_;

  // p_NoBcm_N_initial_ is Bcm's initial position from No, expressed in N, i.e.,
  // x, y, z, the Nx, Ny, Nz measures of Bcm's position vector from point No
  // (World origin).  Note: Bcm (B's center of mass) is coincident with Bo.
  Vector3d p_NoBcm_N_initial_;

  // v_NBcm_B_initial_ is Bcm's initial velocity in N, expressed in B.
  // Note: v_NBcm_B is not (in general) the time-derivative of ẋ, ẏ, ż.
  Vector3d v_NBcm_B_initial_;

  // uniform_gravity_expressed_in_world_ is the local planet's (e.g., Earth)
  // uniform gravitational acceleration, expressed in World (e.g., Earth).
  Vector3d uniform_gravity_expressed_in_world_;
};


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
       const TorqueFreeCylinderExactSolution& torque_free_cylinder_solution,
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
  EXPECT_TRUE(math::IsBothQuaternionAndQuaternionDtOK(quat_NB_drake,
                                                quatDt_NB_drake, 16*tolerance));

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
          const TorqueFreeCylinderExactSolution& torque_free_cylinder_solution,
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
    TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(rigid_body_plant,
      *context, torque_free_cylinder_solution, tolerance, stateDt_drake);

    const double t = context->get_time();
    if (t >= t_final_minus_epsilon) break;

    const double dt = (t + dt_max > t_final) ? (t_final - t) : dt_max;
    rk3.StepOnceAtMost(dt, dt, dt);    // Step forward by at most dt.
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
    const TorqueFreeCylinderExactSolution& torque_free_cylinder_solution,
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
      *(context->get_mutable_continuous_state_vector());
  state_drake.SetFromVector(state_initial);

  // Ensure the time stored by context is set to 0.0 (initial value).
  context->set_time(0.0);

  // Test Drake's calculated values for time-derivative of state at time t = 0
  // versus exact analytical (closed-form) solution.
  // Initially (t = 0), Drake's results should be close to machine-precision.
  const double epsilon = std::numeric_limits<double>::epsilon();
  TestDrakeSolutionVsExactSolutionForTorqueFreeCylinder(rigid_body_plant,
     *context, torque_free_cylinder_solution, 50*epsilon, stateDt_drake);

  // Maybe numerically integrate.
  if (should_numerically_integrate) {
    const double dt_max = 0.2, t_final = 10.0;
    const double maximum_absolute_error_per_integration_step = 1.0E-3;

    // Integrate forward, testing Drake's results vs. exact solution frequently.
    IntegrateForwardWithVariableStepRungeKutta3(rigid_body_plant, dt_max,
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
  DRAKE_DEMAND(context != NULL  &&  stateDt_drake != NULL);

  // Store initial values in a class that can calculate an exact solution.
  // Store gravitational acceleration expressed in N (e.g. [0, 0, -9.8]).
  // Note: a_grav is improper public member of RigidBodyTree class (BAD).
  Quaterniond quat_NB_initial(1, 0, 0, 0);
  const Vector3d w_NB_B_initial(2.0, 4.0, 6.0);
  const Vector3d p_NoBcm_N_initial(1.0, 2.0, 3.0);
  const Vector3d v_NBcm_B_initial(-4.2, 5.5, 6.1);
  const RigidBodyTree<double>& tree = rigid_body_plant.get_rigid_body_tree();
  const Vector3d gravity = tree.a_grav.tail<3>();
  TorqueFreeCylinderExactSolution torque_free_cylinder_exact(quat_NB_initial,
                 w_NB_B_initial, p_NoBcm_N_initial, v_NBcm_B_initial, gravity);

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
  const std::string urdf_dir = "/multibody/benchmarks/"
      "cylinder_torque_free_analytical_solution/";
  const std::string urdf_dir_file_name = GetDrakePath() + urdf_dir + urdf_name;

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
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat_AB    | Quaternion relating Ax, Ay, Az to Bx, By, Bz.
 *            | Note: quat_AB is analogous to the rotation matrix R_AB.
 * w_NB_B     | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
 * wDt_NB_B   | B's angular acceleration in N, expressed in B, [ẇx, ẇy, ẇz].
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
std::tuple<Quaterniond, Vector3d, Vector3d>
TorqueFreeCylinderExactSolution::
CalculateExactRotationalSolutionABInitiallyAligned(const double t) const {
  // Constant values of moments of inertia.
  const double I = 0.04;
  const double J = 0.02;

  // Initial values of wx, wy, wz.
  const Vector3d& w_NB_B_initial = get_w_NB_B_initial();
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
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-------------------------------------------------
 * quat_NB    | Quaternion relating Nx, Ny, Nz to Bx, By, Bz: [e0, e1, e2, e3].
 *            | Note: quat_NB is analogous to the rotation matrix R_NB.
 * quatDt     | Time-derivative of `quat_NB', i.e., [ė0, ė1, ė2, ė3].
 * w_NB_B     | B's angular velocity in N, expressed in B, e.g., [wx, wy, wz].
 * wDt_NB_B   | B's angular acceleration in N, expressed in B, [ẇx, ẇy, ẇz].
 *
 * - [Kane, 1983] "Spacecraft Dynamics," McGraw-Hill Book Co., New York, 1983.
 *   (with P. W. Likins and D. A. Levinson).  Available for free .pdf download:
 *   https://ecommons.cornell.edu/handle/1813/637
 */
std::tuple<Quaterniond, Vector4d, Vector3d, Vector3d>
TorqueFreeCylinderExactSolution::CalculateExactRotationalSolutionNB(
                                 const double t) const {
  // Kane's analytical solution is for quaternion quat_AB that relates A to B,
  // where A is another set Ax, Ay, Az of right-handed orthogonal unit vectors
  // which are fixed in N (Newtonian frame) but initially aligned to Bx, By, Bz.
  Quaterniond quat_AB;
  Vector3d w_NB_B, wDt_NB_B;
  std::tie(quat_AB, w_NB_B, wDt_NB_B) =
      CalculateExactRotationalSolutionABInitiallyAligned(t);

  // Define A basis as World-fixed basis aligned with B's initial orientation.
  // Multiply (Hamilton product) the quaternions quat_NA * quat_AB to produce
  // quat_NB, which is analogous to multiplying rotation matrices R_NA * R_AB
  // to produce the rotation matrix R_NB.
  // In other words, account for quat_NB_initial (which is quat_NA) to calculate
  // the quaternion quat_NB that is analogous to the R_NB rotation matrix.
  const Quaterniond& quat_NA = get_quat_NB_initial();
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
 * @returns Machine-precision values at time t are returned as defined below.
 *
 * std::tuple | Description
 * -----------|-----------------------------------------------------------
 * xyz        | Vector3d [x, y, z], Bcm's position from No, expressed in N.
 * xyzDt      | Vector3d [ẋ, ẏ, ż]  Bcm's velocity in N, expressed in N.
 * xyzDDt     | Vector3d [ẍ  ÿ  z̈], Bcm's acceleration in N, expressed in N.
 */
std::tuple<Vector3d, Vector3d, Vector3d>
TorqueFreeCylinderExactSolution::CalculateExactTranslationalSolution(
                                 const double t) const {
  // Initial values of x, y, z and ẋ, ẏ, ż.
  const Vector3d& xyz_initial = get_p_NoBcm_N_initial();
  const Vector3d& xyzDt_initial =
                  GetInitialVelocityOfBcmInWorldExpressedInWorld();
  const double x0 = xyz_initial[0];
  const double y0 = xyz_initial[1];
  const double z0 = xyz_initial[2];
  const double xDt0 = xyzDt_initial[0];
  const double yDt0 = xyzDt_initial[1];
  const double zDt0 = xyzDt_initial[2];

  // Analytical solution for ẋ, ẏ, z̈ (only gravitational forces on body).
  const Vector3d& gravity = get_uniform_gravity_expressed_in_world();
  const double x_acceleration = gravity[0];
  const double y_acceleration = gravity[1];
  const double z_acceleration = gravity[2];

  // Analytical solution for ẋ, ẏ, ż (since acceleration is constant).
  const double xDt = xDt0 + x_acceleration * t;
  const double yDt = yDt0 + y_acceleration * t;
  const double zDt = zDt0 + z_acceleration * t;

  // Analytical solution for x, y, z (since acceleration is constant).
  const double x = x0 + xDt0*t + 0.5 * x_acceleration * t * t;
  const double y = y0 + yDt0*t + 0.5 * y_acceleration * t * t;
  const double z = z0 + zDt0*t + 0.5 * z_acceleration * t * t;

  // Create a tuple to package for returning.
  std::tuple<Vector3d, Vector3d, Vector3d> returned_tuple;
  Vector3d& xyz    = std::get<0>(returned_tuple);
  Vector3d& xyzDt  = std::get<1>(returned_tuple);
  Vector3d& xyzDDt = std::get<2>(returned_tuple);

  // Fill returned_tuple with results.
  xyz    << x, y, z;
  xyzDt  << xDt, yDt, zDt;
  xyzDDt << x_acceleration, y_acceleration, z_acceleration;

  return returned_tuple;
}


}  // namespace cylinder_torque_free_analytical_solution
}  // namespace benchmarks
}  // namespace drake
