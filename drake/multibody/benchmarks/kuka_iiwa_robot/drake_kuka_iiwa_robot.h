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

// Utility method for creating a transform from frame A to frame B.
// @param[in] R_AB Rotation matrix relating Ax, Ay, Az to Bx, By, Bz.
// @param[in] p_AoBo_A Position vector from Ao to Bo, expressed in A.
// @return Transform relating frame A to frame B.
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

// This class is a MultibodyTree model for a 7-DOF Kuka iiwa robot arm.
// It is used to compare Drake results for the robot end-effector (rigid linkG)
// relative to world (Newtonian frame linkN) versus MotionGenesis solution.
// This class takes input values for each of the 7 joint angles as well as their
// 1st and 2nd time derivatives (e.g., q, q̇, q̈) and calculates the end-effector
// rotation matrix (relative to world), position (from world origin) angular
// velocity, velocity, angular acceleration, and acceleration.
// Geometrical and connectivity data is in file kuka_iiwa_robot.urdf.
class DrakeKukaIIwaRobot {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeKukaIIwaRobot)

  // Construct a 7-DOF Kuka iiwa robot arm (from file kuka_iiwa_robot.urdf).
  DrakeKukaIIwaRobot() {
    // Create a mostly empty MultibodyTree (it has a built-in "world" body).
    // Newtonian reference frame (linkN) is the world body.
    model_ = std::make_unique<MultibodyTree<double>>();
    linkN_ = &(model_->get_world_body());

    // Create a NaN SpatialInertia to instantiate all the links in this robot -
    // which is OK since only kinematic tests are performed (no test of force,
    // torque, statics, momentum, kinetic energy, etc.).  M_Bo_B is a rigid
    // body B's spatial inertia about Bo (B's origin), expressed in B.
    SpatialInertia<double> M_Bo_B;

    // Add this robot's seven links.
    linkA_ = &(model_->AddBody<RigidBody>(M_Bo_B));
    linkB_ = &(model_->AddBody<RigidBody>(M_Bo_B));
    linkC_ = &(model_->AddBody<RigidBody>(M_Bo_B));
    linkD_ = &(model_->AddBody<RigidBody>(M_Bo_B));
    linkE_ = &(model_->AddBody<RigidBody>(M_Bo_B));
    linkF_ = &(model_->AddBody<RigidBody>(M_Bo_B));
    linkG_ = &(model_->AddBody<RigidBody>(M_Bo_B));

    // Create a revolute joint between linkN (Newtonian frame/world) and linkA
    // using two joint-frames, namely "NA" and "AN".  The "inboard frame" NA is
    // welded to linkN and the "outboard frame" AN is welded to linkA.
    // The orientation and position of NA relative to linkN are specified by the
    // second and third arguments in the following method, namely with SpaceXYZ
    // angles and a position vector. Alternately, frame AN is regarded as
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

  // This method sets the Kuka's joint angles and their 1st and 2nd derivatives.
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

  // This method calculates kinematic properties of the end-effector (herein
  // denoted as rigid body G) of a 7-DOF KUKA LBR iiwa robot (14 kg payload).
  // Right-handed orthogonal unit vectors Nx, Ny, Nz are fixed in N (Earth)
  // with Nz vertically upward and right-handed orthogonal unit vectors
  // Gx, Gy, Gz are fixed in G.  The origin of frame N (Earth) is denoted No.
  // The origin Go of end-effector G is located at G's inboard revolute joint.
  //
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] qDt 1st-time-derivative of q (q̇).
  //
  // @returns values defined below.
  //
  // std::tuple | Description
  // -----------|-------------------------------------------------
  // R_NG       | Rotation matrix relating Nx, Ny, Nz to Gx, Gy, Gz.
  // p_NoGo_N   | Go's position from No, expressed in N.
  // w_NG_N     | G's angular velocity in N, expressed in N.
  // v_NGo_N    | Go's velocity in N, expressed in N.
  // alpha_NG_N | G's angular acceleration in N, expressed in N.
  // a_NGo_N    | Go's acceleration in N, expressed in N.
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
    const Eigen::Isometry3d& X_NG = get_body_pose_in_world(pc, *linkG_);

    // Retrieve end-effector spatial velocity from velocity kinematics cache.
    model_->CalcVelocityKinematicsCache(*context_, pc, &vc);
    const SpatialVelocity<double>& V_NG_N =
        get_body_spatial_velocity_in_world(vc, *linkG_);

    // Retrieve end-effector spatial acceleration from acceleration cache.
    model_->CalcAccelerationKinematicsCache(*context_, pc, vc, qDDt, &ac);
    const SpatialAcceleration<double>& A_NG_N =
        get_body_spatial_acceleration_in_world(ac, *linkG_);

    // Create tuple to return results.
    const Eigen::Matrix3d R_NG = X_NG.linear();
    const Eigen::Vector3d p_NoGo_N = X_NG.translation();
    const Eigen::Vector3d w_NG_N = V_NG_N.rotational();
    const Eigen::Vector3d v_NGo_N = V_NG_N.translational();
    const Eigen::Vector3d alpha_NG_N = A_NG_N.rotational();
    const Eigen::Vector3d a_NG_N = A_NG_N.translational();
    return std::make_tuple(R_NG, p_NoGo_N, w_NG_N, v_NGo_N, alpha_NG_N, a_NG_N);
  }

  // Helper method to extract a pose from the position kinematics.
  // TODO(amcastro-tri): When cache entries can be placed in the context,
  // replace by method Body<T>::get_pose_in_world(const systems::Context<T>&).
  const Eigen::Isometry3d& get_body_pose_in_world(
      const PositionKinematicsCache<double>& pc,
      const Body<double>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return pc.get_X_WB(topology.get_body(body.get_index()).body_node);
  }

  // Helper method to extract a SpatialVelocity from the velocity kinematics.
  // TODO(amcastro-tri): When cache entries can be placed in context, replace by
  // method Body<T>::get_spatial_velocity_in_world(const systems::Context<T>&).
  const SpatialVelocity<double>& get_body_spatial_velocity_in_world(
      const VelocityKinematicsCache<double>& vc,
      const Body<double>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return vc.get_V_WB(topology.get_body(body.get_index()).body_node);
  }

  // Helper method to extract SpatialAcceleration from acceleration kinematics.
  // TODO(amcastro-tri): When cache entries can be placed in context, replace by
  // method Body<T>::get_spatial_acceleration_in_world(const Context<T>&).
  const SpatialAcceleration<double>& get_body_spatial_acceleration_in_world(
      const AccelerationKinematicsCache<double>& ac,
      const Body<double>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return ac.get_A_WB(topology.get_body(body.get_index()).body_node);
  }

 protected:
  // Method to add revolute joint (mobilizer) from Body A to Body B.
  // @param[in] A     Mobilizer's inboard  body (frame AB will be welded to A).
  // @param[in] X_AAB Transform relating body A to frame AB.
  // @param[in] B     Mobilizer's outboard body (frame BA will be welded to B).
  // @param[in] X_BBA Transform relating body B to frame BBA.
  // @param[in] revolute_unit_vector  Unit vector orienting the revolute joint.
  // @return RevoluteMobilizer from frame AB on Body A to frame BA on Body B.
  const RevoluteMobilizer<double>& AddRevoluteMobilizer(
      const Body<double>& A, const Eigen::Isometry3d& X_AAB,
      const Body<double>& B, const Eigen::Isometry3d& X_BBA,
      const Eigen::Vector3d &revolute_unit_vector) {
    // Add a FixedOffsetFrame AB to Body A (AB is mobilizer's inboard frame).
    const FixedOffsetFrame<double>& AB =
        model_->AddFrame<FixedOffsetFrame>(A, X_AAB);

    // Add a FixedOffsetFrame BA to Body B (BA is mobilizer's outboard frame).
    const FixedOffsetFrame<double>& BA =
        model_->AddFrame<FixedOffsetFrame>(B, X_BBA);

    // Return a new RevoluteMobilizer between inboard frame and outboard frame.
    return model_->AddMobilizer<RevoluteMobilizer>(AB, BA,
                                                   revolute_unit_vector);
  }

  // Method to add revolute joint (mobilizer) from Body A to Body B.
  // @param[in] A     Mobilizer's inboard  body (frame AB will be welded to A).
  // @param[in] q123A SpaceXYZ angles describing the rotation matrix relating
  //                  unit vectors Ax, Ay, Az to soon-to-be created frame AB.
  // @param[in] xyzA  Ax, Ay, Az measures of the position from Ao to ABo.
  // @param[in] B     Mobilizer's outboard body (frame BA will be welded to B).
  // @param[in] revolute_unit_vector  Unit vector orienting the revolute joint.
  // @return RevoluteMobilizer from frame AB on Body A to frame BA on Body B.
  const RevoluteMobilizer<double> &AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
      const Body<double>& A, const Vector3d& q123A, const Vector3d& xyzA,
      const Body<double>& B, const Vector3d& revolute_unit_vector) {
    // Create transform from inboard body A to mobilizer inboard frame AB.
    const Eigen::Isometry3d X_AAB = MakeIsometry3d(math::rpy2rotmat(q123A),
                                                   xyzA);

    // Create transform from outboard body B to mobilizer outboard frame BA.
    const Eigen::Isometry3d X_BBA = MakeIsometry3d(Eigen::Matrix3d::Identity(),
                                                   Vector3d(0, 0, 0));

    return AddRevoluteMobilizer(A, X_AAB, B, X_BBA, revolute_unit_vector);
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

  // After model is finalized, create default context.
  std::unique_ptr<systems::Context<double>> context_;
};

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
