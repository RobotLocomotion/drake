#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <tuple>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/test_utilities/spatial_kinematics.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

using Eigen::Vector3d;
using multibody_tree::test_utilities::SpatialKinematicsPVA;

/// Utility struct to assist with returning joint torques/forces.
/// --------|----------------------------------------------------------
/// F_Ao_W  | Spatial force on Ao from W, expressed in frame W (world).
/// F_Bo_W  | Spatial force on Bo from A, expressed in frame W (world).
/// F_Co_W  | Spatial force on Co from B, expressed in frame W (world).
/// F_Do_W  | Spatial force on Do from C, expressed in frame W (world).
/// F_Eo_W  | Spatial force on Eo from D, expressed in frame W (world).
/// F_Fo_W  | Spatial force on Fo from E, expressed in frame W (world).
/// F_Go_W  | Spatial force on Go from F, expressed in frame W (world).
template <typename T>
struct KukaRobotJointReactionForces {
  SpatialForce<T> F_Ao_W;
  SpatialForce<T> F_Bo_W;
  SpatialForce<T> F_Co_W;
  SpatialForce<T> F_Do_W;
  SpatialForce<T> F_Eo_W;
  SpatialForce<T> F_Fo_W;
  SpatialForce<T> F_Go_W;
};

/// Utility method for creating a transform from frame A to frame B.
/// @param[in] R_AB Rotation matrix relating Ax, Ay, Az to Bx, By, Bz.
/// @param[in] p_AoBo_A Position vector from Ao to Bo, expressed in A.
/// @retval X_AB Tranform relating frame A to frame B.
Eigen::Isometry3d MakeIsometry3d(const Eigen::Matrix3d& R_AB,
                                 const Eigen::Vector3d& p_AoBo_A) {
  // Initialize all of X_AB (may be more than just linear and translation).
  Eigen::Isometry3d X_AB = Eigen::Isometry3d::Identity();
  // X_AB.linear() returns a mutable references to the 3x3 rotation matrix
  // part of X_AB.  X_AB.translation() returns a mutable reference to the
  // 3x1 position vector part of X_AB.
  X_AB.linear() = R_AB;
  X_AB.translation() = p_AoBo_A;
  return X_AB;
}

/// This class is a MultibodyTree model for a 7-DOF Kuka iiwa robot arm.
/// It is used to compare Drake results for the robot end-effector (rigid linkG)
/// relative to world (Newtonian frame linkN) versus MotionGenesis solution.
/// This class takes input values for each of the 7 joint angles (q) as well as
/// their 1st and 2nd time derivatives (q̇, q̈) and calculates the end-effector
/// rotation matrix (relative to world), position (from world origin) angular
/// velocity, velocity, angular acceleration, and acceleration.
/// Geometrical and connectivity data is in file kuka_iiwa_robot.urdf.
template <typename T>
class DrakeKukaIIwaRobot {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeKukaIIwaRobot)

  /// Construct a 7-DOF Kuka iiwa robot arm (from file kuka_iiwa_robot.urdf).
  /// The robot is constructed with 7 revolute joints.
  /// @param[in] gravity Earth's gravitational acceleration in m/s².  The world
  /// z-unit vector is vertically upward.  If a gravity value of 9.8 is passed
  /// to this constructor, it means the gravity vector is directed opposite the
  /// world upward z-unit vector (which is correct -- gravity is downward).
  explicit DrakeKukaIIwaRobot(double gravity) {
    // Create a mostly empty MultibodyTree (it has a built-in "world" body).
    // Newtonian reference frame (linkN) is the world body.
    model_ = std::make_unique<MultibodyTree<T>>();
    linkN_ = &(model_->get_world_body());

    // Create SpatialInertia for each link in this robot. M_Bo_B designates a
    // rigid body B's spatial inertia about Bo (B's origin), expressed in B.
    const SpatialInertia<double> M_AAo_A =
        SpatialInertia<double>::MakeFromCentralInertia(massA_, p_AoAcm_A_,
                                                       I_AAcm_A_);
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::MakeFromCentralInertia(massB_, p_BoBcm_B_,
                                                       I_BBcm_B_);
    const SpatialInertia<double> M_CCo_C =
        SpatialInertia<double>::MakeFromCentralInertia(massC_, p_CoCcm_C_,
                                                       I_CCcm_C_);
    const SpatialInertia<double> M_DDo_D =
        SpatialInertia<double>::MakeFromCentralInertia(massD_, p_DoDcm_D_,
                                                       I_DDcm_D_);
    const SpatialInertia<double> M_EEo_E =
        SpatialInertia<double>::MakeFromCentralInertia(massE_, p_EoEcm_E_,
                                                       I_EEcm_E_);
    const SpatialInertia<double> M_FFo_F =
        SpatialInertia<double>::MakeFromCentralInertia(massF_, p_FoFcm_F_,
                                                       I_FFcm_F_);
    const SpatialInertia<double> M_GGo_G =
        SpatialInertia<double>::MakeFromCentralInertia(massG_, p_GoGcm_G_,
                                                       I_GGcm_G_);

    // Add this robot's seven links.
    linkA_ = &(model_->template AddBody<RigidBody>(M_AAo_A));
    linkB_ = &(model_->template AddBody<RigidBody>(M_BBo_B));
    linkC_ = &(model_->template AddBody<RigidBody>(M_CCo_C));
    linkD_ = &(model_->template AddBody<RigidBody>(M_DDo_D));
    linkE_ = &(model_->template AddBody<RigidBody>(M_EEo_E));
    linkF_ = &(model_->template AddBody<RigidBody>(M_FFo_F));
    linkG_ = &(model_->template AddBody<RigidBody>(M_GGo_G));

    // Create a revolute joint between linkN (Newtonian frame/world) and linkA
    // using two joint-frames, namely "Na" and "An".  The "inboard frame" Na is
    // welded to linkN and the "outboard frame" An is welded to linkA.
    // The orientation and position of Na relative to linkN are specified by the
    // second and third arguments in the following method, namely with SpaceXYZ
    // angles and a position vector. Alternately, frame An is regarded as
    // coincident with linkA.
    NA_mobilizer_ = &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
        *linkN_, Vector3d(0, 0, 0), Vector3d(0, 0, 0.1575),
        *linkA_, Eigen::Vector3d::UnitZ());

    // Create a revolute joint between linkA and linkB.
    AB_mobilizer_ = &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
        *linkA_, Vector3d(M_PI_2, 0, M_PI), Vector3d(0, 0, 0.2025),
        *linkB_, Eigen::Vector3d::UnitZ());

    // Create a revolute joint between linkB and linkC.
    BC_mobilizer_ = &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
        *linkB_, Vector3d(M_PI_2, 0, M_PI), Vector3d(0, 0.2045, 0),
        *linkC_, Eigen::Vector3d::UnitZ());

    // Create a revolute joint between linkB and linkC.
    CD_mobilizer_ = &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
        *linkC_, Vector3d(M_PI_2, 0, 0), Vector3d(0, 0, 0.2155),
        *linkD_, Eigen::Vector3d::UnitZ());

    // Create a revolute joint between linkD and linkE.
    DE_mobilizer_ = &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
        *linkD_, Vector3d(-M_PI_2, M_PI, 0), Vector3d(0, 0.1845, 0),
        *linkE_, Eigen::Vector3d::UnitZ());
    // Create a revolute joint between linkE and linkF.
    EF_mobilizer_ = &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
        *linkE_, Vector3d(M_PI_2, 0, 0), Vector3d(0, 0, 0.2155),
        *linkF_, Eigen::Vector3d::UnitZ());

    // Create a revolute joint between linkE and linkF.
    FG_mobilizer_ = &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
        *linkF_, Vector3d(-M_PI_2, M_PI, 0), Vector3d(0, 0.081, 0),
        *linkG_, Eigen::Vector3d::UnitZ());

    // Add force element for a constant gravity pointing downwards, that is, in
    // the negative z-axis direction.
    set_gravity(gravity);
    const Eigen::Vector3d gravity_vector = -gravity_ * Eigen::Vector3d::UnitZ();
    model_->template AddForceElement<UniformGravityFieldElement>(
        gravity_vector);

    // Finalize() stage sets the topology (model is built).
    model_->Finalize();

    // After Finalize() method has been called, Context can be created.
    context_ = model_->CreateDefaultContext();
  }

  /// This method gets the number of rigid bodies in this robot.
  /// @returns the number of rigid bodies in this robot.
  int get_number_of_rigid_bodies() const  { return model_->get_num_bodies(); }

  /// This method calculates kinematic properties of the end-effector (herein
  /// denoted as rigid body G) of a 7-DOF KUKA LBR iiwa robot (14 kg payload).
  /// Right-handed orthogonal unit vectors Nx, Ny, Nz are fixed in N (Earth)
  /// with Nz vertically upward and right-handed orthogonal unit vectors
  /// Gx, Gy, Gz are fixed in G.  The origin of frame N (Earth) is denoted No.
  /// The origin Go of end-effector G is located at G's inboard revolute joint.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] qDt 1st-time-derivative of q.
  /// @param[in] qDDt 2nd-time-derivative of q.
  ///
  /// @returns G's kinematics in N, expressed in N.
  SpatialKinematicsPVA<T>
  CalcEndEffectorKinematics(const Eigen::Ref<const VectorX<T>>& q,
                            const Eigen::Ref<const VectorX<T>>& qDt,
                            const Eigen::Ref<const VectorX<T>>& qDDt) {
    SetJointAnglesAnd1stDerivatives(q.data(), qDt.data());

    // For each body, set the pose and spatial velocity in the position,
    // velocity, and acceleration caches with specified values for testing.
    PositionKinematicsCache<T> pc(model_->get_topology());
    VelocityKinematicsCache<T> vc(model_->get_topology());
    AccelerationKinematicsCache<T> ac(model_->get_topology());

    // Retrieve end-effector pose from position kinematics cache.
    model_->CalcPositionKinematicsCache(*context_, &pc);
    const Isometry3<T>& X_NG = linkG_->get_pose_in_world(pc);

    // Retrieve end-effector spatial velocity from velocity kinematics cache.
    model_->CalcVelocityKinematicsCache(*context_, pc, &vc);
    const SpatialVelocity<T>& V_NG_N =
        linkG_->get_spatial_velocity_in_world(vc);

    // Retrieve end-effector spatial acceleration from acceleration cache.
    model_->CalcAccelerationKinematicsCache(*context_, pc, vc, qDDt, &ac);
    const SpatialAcceleration<T>& A_NG_N =
        linkG_->get_spatial_acceleration_in_world(ac);

    // Create a class to return the results.
    return SpatialKinematicsPVA<T>(X_NG, V_NG_N, A_NG_N);
  }

  /// Given a set of points `Pi` with position vectors `p_GPi` in the end
  /// effector frame G, this method computes the geometric Jacobian `J_NGpi`,
  /// in the Newtonian (world) frame N defined by:
  /// <pre>
  ///   J_NGpi(q) = d(v_NGpi(q, v))/dv
  /// </pre>
  /// where `v_NGpi` is the translational velocity of point `Pi` in the world
  /// frame N as it moves with the end effector G and v is the vector of
  /// generalized velocities. Since the velocity of each point `Pi` is linear
  /// in the generalized velocities, the geometric Jacobian `J_NGpi` is a
  /// function of the generalized coordinates q only.
  ///
  /// The position of each point `Pi` in the set is specified by its (fixed)
  /// position `p_GPi` in the end effector frame G.
  ///
  /// @param[in] q
  ///   The vector of generalized positions.
  /// @param[in] p_GPi
  ///   A matrix with the fixed position of a set of points `Pi` measured and
  ///   expressed in the end effector frame G.
  ///   Each column of this matrix contains the position vector `p_GPi` for a
  ///   point `Pi` measured and expressed in frame G. Therefore this input
  ///   matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the set.
  /// @param[out] p_NGpi
  ///   The output positions of each point `Pi` now computed as measured and
  ///   expressed in the world frame N.
  ///   The output `p_NGpi` **must** have the same size as the input set
  ///   `p_GPi` or otherwise this method throws a std::runtime_error exception.
  ///   That is `p_NGpi` **must** be in `ℝ³ˣⁿᵖ`.
  /// @param[out] J_NGpi
  ///   The geometric Jacobian, function of the generalized positions
  ///   q only. This Jacobian relates the translational velocity `v_NGpi` of
  ///   each point `Qi` in the input set in solidary motion with frame G by:
  ///   <pre>
  ///     `v_NGpi(q, v) = J_NGpi(q)⋅v`
  ///   </pre>
  ///   so that `v_NGpi` is a column vector of size `3⋅np` concatenating the
  ///   velocity of all points `Pi` in the same order they were given in the
  ///   input set. Therefore `J_NGpi` is a matrix of size `3⋅np x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `J_NGpi` **must**
  ///   have size `3⋅np x nv` or this method throws a std::runtime_error
  ///   exception.
  void CalcPointsOnEndEffectorGeometricJacobian(
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const MatrixX<T>>& p_GPi,
      EigenPtr<MatrixX<T>> p_NGpi, EigenPtr<MatrixX<T>> J_NGpi) {
    // Both, p_NGpi and J_NGpi are functions of the generalized positions q,
    // only. Therefore we arbitrarily set v = 0.
    const VectorX<T> v = VectorX<T>::Zero(7);
    SetJointAnglesAnd1stDerivatives(q.data(), v.data());
    model_->CalcPointsGeometricJacobianExpressedInWorld(
        *context_, linkG_->get_body_frame(), p_GPi, p_NGpi, J_NGpi);
  }

  /// This method calculates joint reaction torques/forces for a 7-DOF KUKA iiwa
  /// robot, from known joint angles and their 1st and 2nd time-derivatives.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] qDt 1st-time-derivative of q.
  /// @param[in] qDDt 2nd-time-derivative of q.
  ///
  /// @returns a structure holding the quantities defined below.
  /// --------|----------------------------------------------------------
  /// F_Ao_W  | Spatial force on Ao from W, expressed in frame W (world).
  /// F_Bo_W  | Spatial force on Bo from A, expressed in frame W (world).
  /// F_Co_W  | Spatial force on Co from B, expressed in frame W (world).
  /// F_Do_W  | Spatial force on Do from C, expressed in frame W (world).
  /// F_Eo_W  | Spatial force on Eo from D, expressed in frame W (world).
  /// F_Fo_W  | Spatial force on Fo from E, expressed in frame W (world).
  /// F_Go_W  | Spatial force on Go from F, expressed in frame W (world).
  KukaRobotJointReactionForces<T> CalcJointReactionForces(
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& qDt,
      const Eigen::Ref<const VectorX<T>>& qDDt) {
    SetJointAnglesAnd1stDerivatives(q.data(), qDt.data());

    // Get the position, velocity, and acceleration cache from the context.
    PositionKinematicsCache<T> pc(model_->get_topology());
    VelocityKinematicsCache<T> vc(model_->get_topology());
    AccelerationKinematicsCache<T> ac(model_->get_topology());
    model_->CalcPositionKinematicsCache(*context_, &pc);
    model_->CalcVelocityKinematicsCache(*context_, pc, &vc);
    model_->CalcAccelerationKinematicsCache(*context_, pc, vc, qDDt, &ac);

    // Input vector of generalized forces for known applied force/torques,
    // e.g., gravity, known models of visous friction, etc.
    const int number_of_generalized_speeds = model_->get_num_velocities();
    VectorX<T> generalized_force_applied(number_of_generalized_speeds);

    // Input vector of spatial forces for known applied force/torques,
    // e.g., gravity, known models of visous friction, etc.
    const int number_of_bodies = get_number_of_rigid_bodies();
    std::vector<SpatialForce<T>> Fapplied_Bo_W_array(number_of_bodies);

    // Fill arrays generalized_force_applied and F_Bo_W_array using the fact
    // that gravity was included earlier in the model via:
    // model_->AddForceElement<UniformGravityFieldElement>(gravity_vector);
    model_->CalcForceElementsContribution(
        *context_, pc, vc, &Fapplied_Bo_W_array, &generalized_force_applied);

    // Output vector of generalized forces for calculated motor torques
    // required to drive the Kuka robot at its specified rate.
    VectorX<T> generalized_force_output(number_of_generalized_speeds);

    // Output vector of spatial forces for joint reaction force/torques for
    // each body B at their inboard frame Mo, expressed in the world W.
    std::vector<SpatialForce<T>> F_BMo_W_array(number_of_bodies);

    // Output vector of generalized forces associated with the motor torques
    // required to drive the Kuka robot at its specified rate.
    std::vector<SpatialAcceleration<T>> A_WB_array(number_of_bodies);

    // Calculate inverse dynamics on this robot.
    model_->CalcInverseDynamics(*context_, pc, vc, qDDt,
                Fapplied_Bo_W_array, generalized_force_applied,
                &A_WB_array, &F_BMo_W_array, &generalized_force_output);

    // Put joint reaction forces into return struct.
    KukaRobotJointReactionForces<T> forces;
    forces.F_Ao_W = F_BMo_W_array[linkA_->get_node_index()];
    forces.F_Bo_W = F_BMo_W_array[linkB_->get_node_index()];
    forces.F_Co_W = F_BMo_W_array[linkC_->get_node_index()];
    forces.F_Do_W = F_BMo_W_array[linkD_->get_node_index()];
    forces.F_Eo_W = F_BMo_W_array[linkE_->get_node_index()];
    forces.F_Fo_W = F_BMo_W_array[linkF_->get_node_index()];
    forces.F_Go_W = F_BMo_W_array[linkG_->get_node_index()];
    return forces;
  }

 private:
  // Method to add revolute joint (mobilizer) from Body A to Body B.
  // @param[in] A     Mobilizer's inboard  body (frame Ab will be welded to A).
  // @param[in] X_AAb Transform relating body A to frame Ab.
  // @param[in] B     Mobilizer's outboard body (frame Ba will be welded to B).
  // @param[in] X_BBa Transform relating body B to frame Ba.
  // @param[in] revolute_unit_vector  Unit vector expressed in frame Ab that
  //         characterizes a positive rotation of Ba from Ab (right-hand-rule).
  // @return RevoluteMobilizer from frame Ab on Body A to frame Ba on Body B.
  const RevoluteMobilizer<T>& AddRevoluteMobilizer(
      const Body<T>& A, const Isometry3<double>& X_AAb,
      const Body<T>& B, const Isometry3<double>& X_BBa,
      const Vector3<double>& revolute_unit_vector) {
    // Add a FixedOffsetFrame Ab to Body A (Ab is mobilizer's inboard frame).
    const FixedOffsetFrame<T>& Ab =
        model_->template AddFrame<FixedOffsetFrame>(A, X_AAb);

    // Add a FixedOffsetFrame Ba to Body B (Ba is mobilizer's outboard frame).
    const FixedOffsetFrame<T>& Ba =
        model_->template AddFrame<FixedOffsetFrame>(B, X_BBa);

    // Return a new RevoluteMobilizer between inboard frame and outboard frame.
    return model_->template AddMobilizer<RevoluteMobilizer>(
        Ab, Ba, revolute_unit_vector);
  }

  // Method to add revolute joint (mobilizer) from Body A to Body B.
  // @param[in] A     Mobilizer's inboard  body (frame Ab will be welded to A).
  // @param[in] q123A SpaceXYZ angles describing the rotation matrix relating
  //                  unit vectors Ax, Ay, Az to soon-to-be created frame Ab.
  // @param[in] xyzA  Ax, Ay, Az measures of the position from Ao to Abo.
  // @param[in] B     Mobilizer's outboard body (frame Ba will be welded to B
  //                  so it is coincident with body B's frame). In other words,
  //                  mobilizer's outboard frame Ba will be coincident with
  //                  the outboard body B.
  // @param[in] revolute_unit_vector  Unit vector expressed in frame Ab that
  //         characterizes a positive rotation of Ba from Ab (right-hand-rule).
  // @return RevoluteMobilizer from frame Ab on Body A to frame Ba on Body B.
  const RevoluteMobilizer<T>& AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
      const Body<T>& A,
      const Vector3<double>& q123A, const Vector3<double>& xyzA,
      const Body<T>& B, const Vector3<double>& revolute_unit_vector) {
    // Create transform from inboard body A to mobilizer inboard frame Ab.
    const Eigen::Isometry3d X_AAb = MakeIsometry3d(math::rpy2rotmat(q123A),
                                                   xyzA);

    // Create transform from outboard body B to mobilizer outboard frame Ba.
    const Eigen::Isometry3d X_BBa = MakeIsometry3d(Eigen::Matrix3d::Identity(),
                                                   Vector3d(0, 0, 0));

    return AddRevoluteMobilizer(A, X_AAb, B, X_BBa, revolute_unit_vector);
  }

  // This method sets the Kuka joint angles and their 1st and 2nd derivatives.
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] qDt 1st-time-derivative of q (q̇).
  void SetJointAnglesAnd1stDerivatives(const T q[7],
                                       const T qDt[7]) {
    systems::Context<T>* context = context_.get();

    NA_mobilizer_->set_angle(context, q[0]);
    AB_mobilizer_->set_angle(context, q[1]);
    BC_mobilizer_->set_angle(context, q[2]);
    CD_mobilizer_->set_angle(context, q[3]);
    DE_mobilizer_->set_angle(context, q[4]);
    EF_mobilizer_->set_angle(context, q[5]);
    FG_mobilizer_->set_angle(context, q[6]);

    NA_mobilizer_->set_angular_rate(context, qDt[0]);
    AB_mobilizer_->set_angular_rate(context, qDt[1]);
    BC_mobilizer_->set_angular_rate(context, qDt[2]);
    CD_mobilizer_->set_angular_rate(context, qDt[3]);
    DE_mobilizer_->set_angular_rate(context, qDt[4]);
    EF_mobilizer_->set_angular_rate(context, qDt[5]);
    FG_mobilizer_->set_angular_rate(context, qDt[6]);
  }

  // This method sets Earth's (or astronomical body's) uniform gravitational
  // acceleration ("little g").  By default, little g is initialized to
  // 0.0 m/s² (not 9.81 m/s²).  Right-handed orthogonal unit vectors Nx, Ny, Nz
  // are fixed in N (Earth) with Nz vertically upward (so gravity is in -Nz).
  // @param[in] gravity Earth's gravitational acceleration in m/s².
  void set_gravity(double gravity) { gravity_ = gravity; }

  // This model's MultibodyTree always has a built-in "world" body.
  // Newtonian reference frame (linkN) is the world body.
  std::unique_ptr<MultibodyTree<T>> model_;
  const Body<T>* linkN_;

  // Rigid bodies (robot links).
  const RigidBody<T>* linkA_;
  const RigidBody<T>* linkB_;
  const RigidBody<T>* linkC_;
  const RigidBody<T>* linkD_;
  const RigidBody<T>* linkE_;
  const RigidBody<T>* linkF_;
  const RigidBody<T>* linkG_;

  // Joints (mobilizers).
  const RevoluteMobilizer<T>* NA_mobilizer_;
  const RevoluteMobilizer<T>* AB_mobilizer_;
  const RevoluteMobilizer<T>* BC_mobilizer_;
  const RevoluteMobilizer<T>* CD_mobilizer_;
  const RevoluteMobilizer<T>* DE_mobilizer_;
  const RevoluteMobilizer<T>* EF_mobilizer_;
  const RevoluteMobilizer<T>* FG_mobilizer_;

  // Mass of each link (in kg).
  const double massA_ = 5.76;
  const double massB_ = 6.35;
  const double massC_ = 3.5;
  const double massD_ = 3.5;
  const double massE_ = 3.5;
  const double massF_ = 1.8;
  const double massG_ = 1.2;

  // Position of each body's center of mass from body origin, expressed in body.
  // Example: For a body B with center of mass Bcm and origin Bo, p_BoBcm_B is
  // the position from Bo to Bcm, expressed in terms of Bx, By, Bz (in meters).
  const Vector3d p_AoAcm_A_{0,     -0.03,   0.12};
  const Vector3d p_BoBcm_B_{0.0003, 0.059,  0.042};
  const Vector3d p_CoCcm_C_{0,      0.03,   0.13};
  const Vector3d p_DoDcm_D_{0,      0.067,  0.034};
  const Vector3d p_EoEcm_E_{0.0001, 0.021,  0.076};
  const Vector3d p_FoFcm_F_{0,      0.0006, 0.0004};
  const Vector3d p_GoGcm_G_{0,      0,      0.02};

  // Inertia matrix of each body about its center of mass, expressed in body.
  // Example: For a body B with center of mass Bcm, I_Bcm_B is B's inertia
  // matrix about Bcm, expressed in terms of Bx, By, Bz (in kg * meters^2).
  const RotationalInertia<double> I_AAcm_A_{0.033,  0.0333, 0.0123};
  const RotationalInertia<double> I_BBcm_B_{0.0305, 0.0304, 0.011};
  const RotationalInertia<double> I_CCcm_C_{0.025,  0.0238, 0.0076};
  const RotationalInertia<double> I_DDcm_D_{0.017,  0.0164, 0.006};
  const RotationalInertia<double> I_EEcm_E_{0.01,   0.0087, 0.00449};
  const RotationalInertia<double> I_FFcm_F_{0.0049, 0.0047, 0.0036};
  const RotationalInertia<double> I_GGcm_G_{0.001,  0.001,  0.001};

  // Earth's (or astronomical body's) gravitational acceleration.
  double gravity_ = 0.0;

  // After model is finalized, create default context.
  std::unique_ptr<systems::Context<T>> context_;
};

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
