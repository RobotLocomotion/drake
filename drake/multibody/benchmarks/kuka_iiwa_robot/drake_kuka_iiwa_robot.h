#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <tuple>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

using Eigen::Vector3d;

/// Utility struct to assist with returning joint torques/forces.
/// --------|----------------------------------------------------------
/// F_Ao_W  | Spatial force on Ao from W, expressed in frame W (world).
/// F_Bo_W  | Spatial force on Bo from A, expressed in frame W (world).
/// F_Co_W  | Spatial force on Co from B, expressed in frame W (world).
/// F_Do_W  | Spatial force on Do from C, expressed in frame W (world).
/// F_Eo_W  | Spatial force on Eo from D, expressed in frame W (world).
/// F_Fo_W  | Spatial force on Fo from E, expressed in frame W (world).
/// F_Go_W  | Spatial force on Go from F, expressed in frame W (world).
struct KukaRobotJointReactionForces {
  SpatialForce<double> F_Ao_W;
  SpatialForce<double> F_Bo_W;
  SpatialForce<double> F_Co_W;
  SpatialForce<double> F_Do_W;
  SpatialForce<double> F_Eo_W;
  SpatialForce<double> F_Fo_W;
  SpatialForce<double> F_Go_W;
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
class DrakeKukaIIwaRobot {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeKukaIIwaRobot)

  /// Construct a 7-DOF Kuka iiwa robot arm (from file kuka_iiwa_robot.urdf).
  /// The robot is constructed with 7 revolute joints.
  DrakeKukaIIwaRobot() {
    // Create a mostly empty MultibodyTree (it has a built-in "world" body).
    // Newtonian reference frame (linkN) is the world body.
    model_ = std::make_unique<MultibodyTree<double>>();
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
    linkA_ = &(model_->AddBody<RigidBody>(M_AAo_A));
    linkB_ = &(model_->AddBody<RigidBody>(M_BBo_B));
    linkC_ = &(model_->AddBody<RigidBody>(M_CCo_C));
    linkD_ = &(model_->AddBody<RigidBody>(M_DDo_D));
    linkE_ = &(model_->AddBody<RigidBody>(M_EEo_E));
    linkF_ = &(model_->AddBody<RigidBody>(M_FFo_F));
    linkG_ = &(model_->AddBody<RigidBody>(M_GGo_G));

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

    // Finalize() stage sets the topology (model is built).
    model_->Finalize();

    // After Finalize() method has been called, Context can be created.
    context_ = model_->CreateDefaultContext();
  }

  /// This method gets the number of rigid bodies in this robot.
  /// @returns the number of rigid bodies in this robot.
  int get_number_of_rigid_bodies() const  {return model_->get_num_bodies();}

  /// This method sets Earth's (or astronomical body's) uniform gravitational
  /// acceleration ("little g").  By default, little g is initialized to
  /// 0.0 m/s² (not 9.81 m/s²).  Right-handed orthogonal unit vectors Nx, Ny, Nz
  /// are fixed in N (Earth) with Nz vertically upward (so gravity is in -Nz).
  /// @param[in] gravity Earth's gravitational acceleration in m/s².
  void set_gravity(double gravity) {gravity_ = gravity;}

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
  /// @returns values defined below.
  ///
  /// std::tuple | Description
  /// -----------|-------------------------------------------------
  /// R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  /// p_NoGo_N   | Go's position from No, expressed in N.
  /// w_NG_N     | G's angular velocity in N, expressed in N.
  /// v_NGo_N    | Go's velocity in N, expressed in N.
  /// alpha_NG_N | G's angular acceleration in N, expressed in N.
  /// a_NGo_N    | Go's acceleration in N, expressed in N.
  std::tuple<Eigen::Matrix3d, Vector3d, Vector3d, Vector3d, Vector3d, Vector3d>
  CalcEndEffectorKinematics(const Eigen::Ref<const VectorX<double>>& q,
                            const Eigen::Ref<const VectorX<double>>& qDt,
                            const Eigen::Ref<const VectorX<double>>& qDDt) {
    SetJointAnglesAnd1stDerivatives(q.data(), qDt.data());

    // For each body, set the pose and spatial velocity in the position,
    // velocity, and acceleration caches with specified values for testing.
    PositionKinematicsCache<double> pc(model_->get_topology());
    VelocityKinematicsCache<double> vc(model_->get_topology());
    AccelerationKinematicsCache<double> ac(model_->get_topology());

    // Retrieve end-effector pose from position kinematics cache.
    model_->CalcPositionKinematicsCache(*context_, &pc);
    const Eigen::Isometry3d& X_NG = linkG_->get_pose_in_world(pc);

    // Retrieve end-effector spatial velocity from velocity kinematics cache.
    model_->CalcVelocityKinematicsCache(*context_, pc, &vc);
    const SpatialVelocity<double>& V_NG_N =
        linkG_->get_spatial_velocity_in_world(vc);

    // Retrieve end-effector spatial acceleration from acceleration cache.
    model_->CalcAccelerationKinematicsCache(*context_, pc, vc, qDDt, &ac);
    const SpatialAcceleration<double>& A_NG_N =
        linkG_->get_spatial_acceleration_in_world(ac);

    // Create tuple to return results.
    const Eigen::Matrix3d R_NG = X_NG.linear();
    const Eigen::Vector3d p_NoGo_N = X_NG.translation();
    const Eigen::Vector3d w_NG_N = V_NG_N.rotational();
    const Eigen::Vector3d v_NGo_N = V_NG_N.translational();
    const Eigen::Vector3d alpha_NG_N = A_NG_N.rotational();
    const Eigen::Vector3d a_NG_N = A_NG_N.translational();
    return std::make_tuple(R_NG, p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NG_N);
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
  const KukaRobotJointReactionForces
      CalcJointReactionForces(const Eigen::Ref<const VectorX<double>>& q,
                              const Eigen::Ref<const VectorX<double>>& qDt,
                              const Eigen::Ref<const VectorX<double>>& qDDt) {
    SetJointAnglesAnd1stDerivatives(q.data(), qDt.data());

    // Get the position, velocity, and acceleration cache from the context.
    PositionKinematicsCache<double> pc(model_->get_topology());
    VelocityKinematicsCache<double> vc(model_->get_topology());
    AccelerationKinematicsCache<double> ac(model_->get_topology());
    model_->CalcPositionKinematicsCache(*context_, &pc);
    model_->CalcVelocityKinematicsCache(*context_, pc, &vc);
    model_->CalcAccelerationKinematicsCache(*context_, pc, vc, qDDt, &ac);

    // TODO(mitiguy) Properly calculate joint reaction forces.
    KukaRobotJointReactionForces forces;
    forces.F_Ao_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());
    forces.F_Bo_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());
    forces.F_Co_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());
    forces.F_Do_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());
    forces.F_Eo_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());
    forces.F_Fo_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());
    forces.F_Go_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());
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
  const RevoluteMobilizer<double>& AddRevoluteMobilizer(
      const Body<double>& A, const Eigen::Isometry3d& X_AAb,
      const Body<double>& B, const Eigen::Isometry3d& X_BBa,
      const Eigen::Vector3d& revolute_unit_vector) {
    // Add a FixedOffsetFrame Ab to Body A (Ab is mobilizer's inboard frame).
    const FixedOffsetFrame<double>& Ab =
        model_->AddFrame<FixedOffsetFrame>(A, X_AAb);

    // Add a FixedOffsetFrame Ba to Body B (Ba is mobilizer's outboard frame).
    const FixedOffsetFrame<double>& Ba =
        model_->AddFrame<FixedOffsetFrame>(B, X_BBa);

    // Return a new RevoluteMobilizer between inboard frame and outboard frame.
    return model_->AddMobilizer<RevoluteMobilizer>(Ab, Ba,
                                                   revolute_unit_vector);
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
  const RevoluteMobilizer<double>& AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
      const Body<double>& A, const Vector3d& q123A, const Vector3d& xyzA,
      const Body<double>& B, const Vector3d& revolute_unit_vector) {
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
  void SetJointAnglesAnd1stDerivatives(const double q[7],
                                       const double qDt[7]) {
    systems::Context<double> *context = context_.get();

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

  // This model's MultibodyTree always has a built-in "world" body.
  // Newtonian reference frame (linkN) is the world body.
  std::unique_ptr<MultibodyTree<double>> model_;
  const Body<double>* linkN_;

  // Rigid bodies (robot links).
  const RigidBody<double>* linkA_;
  const RigidBody<double>* linkB_;
  const RigidBody<double>* linkC_;
  const RigidBody<double>* linkD_;
  const RigidBody<double>* linkE_;
  const RigidBody<double>* linkF_;
  const RigidBody<double>* linkG_;

  // Joints (mobilizers).
  const RevoluteMobilizer<double>* NA_mobilizer_;
  const RevoluteMobilizer<double>* AB_mobilizer_;
  const RevoluteMobilizer<double>* BC_mobilizer_;
  const RevoluteMobilizer<double>* CD_mobilizer_;
  const RevoluteMobilizer<double>* DE_mobilizer_;
  const RevoluteMobilizer<double>* EF_mobilizer_;
  const RevoluteMobilizer<double>* FG_mobilizer_;

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
  std::unique_ptr<systems::Context<double>> context_;
};

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
