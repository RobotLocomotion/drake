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

using Eigen::Vector3d;

// Utility method for creating a transform from frame A to frame B.
// @param[in] R_AB Rotation matrix relating Ax, Ay, Az to Bx, By, Bz.
// @param[in] p_AoBo_A Position vector from Ao to Bo, expressed in A.
// @return Transform relating frame A to frame B.
Eigen::Isometry3d CreateIsometry3dFromOrientationAndPosition(
  const Eigen::Matrix3d& R_AB, const Eigen::Vector3d& p_AoBo_A) {
    // Initialize all of X_AB (which may be more than linear and translation).
    Eigen::Isometry3d X_AB = Eigen::Isometry3d::Identity();
    // X_AB.linear() returns a mutable references to the 3x3 rotation matrix
    // part of X_AB.  X_AB.translation() returns a mutable reference to the
    // 3x1 position vector part of X_AB.
    X_AB.linear() = R_AB;
    X_AB.translation() = p_AoBo_A;
  return X_AB;
  }

// Utility method for creating a transform from frame A to frame B.
// @param[in] q123 SpaceXYZ angles describing the rotation matrix relating
//                 unit vectors Ax, Ay, Az to Bx, By, Bz.
// @param[in] xyz  Ax, Ay, Az measures of the position from Ao to Bo.
// @return Transform relating frame A to frame B.
Eigen::Isometry3d CreateIsometry3dFromSpaceXYZAnglesAndXYZ(
  const Vector3d& q123, const Vector3d& xyz) {
    // Form rotation matrix from SpaceXYZ q1, q2, q3 (roll-pitch-yaw) rotation
    // sequence which is equivalent to BodyZYX q3, q2, q1 (yaw-pitch-roll).
    const Eigen::Matrix3d R_AB = math::rpy2rotmat(q123);
    const Eigen::Vector3d p_AoBo_A(xyz);
    return CreateIsometry3dFromOrientationAndPosition(R_AB, p_AoBo_A);
  }

// This class is a MultibodyTree model for a 7-DOF Kuka iiwa robot arm.
// It is used to compare Drake results for the robot end-effector (rigid linkG)
// relative to World (Newtonian frame linkN) versus MotionGenesis solution.
// The tests use an input state (q, qDt) for each of the 7 joints and
// calculates orientation, position, angular velocity, and velocity.
// Geometrical and connectivity data is in file kuka_iiwa_robot.urdf.
class KukaIIwaRobotTestKinematics {
 public:
  // DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KukaIIwaRobotTestKinematics);

  // Construct a 7-DOF Kuka iiwa robot arm (from file kuka_iiwa_robot.urdf).
  KukaIIwaRobotTestKinematics() {
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

    // Create a revolute joint between linkN (Newtonian frame) and linkA using
    // two joint-frames, namely "NA" and "AN".  The "inboard frame" NA is
    // welded to linkN and the "outboard frame" AN is welded to linkA.
    // The orientation and position (pose) of each frame is described below.
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

    // Finalize() stage (model is built) which should validate the topology.
    model_->Finalize();

    // After Finalize(), able to create Context.
    context_ = model_->CreateDefaultContext();
    mbt_context_ = dynamic_cast<MultibodyTreeContext<double>*>(context_.get());
  }

  // This method sets the Kuka robot's joint angles and joint-rates.
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] v time-derivatives of q (generalized speeds).
  void SetJointAnglesAndJointRates(const double q[7], const double v[7]) {
    NA_mobilizer_->set_angle(context_.get(), q[0]);
    AB_mobilizer_->set_angle(context_.get(), q[1]);
    BC_mobilizer_->set_angle(context_.get(), q[2]);
    CD_mobilizer_->set_angle(context_.get(), q[3]);
    DE_mobilizer_->set_angle(context_.get(), q[4]);
    EF_mobilizer_->set_angle(context_.get(), q[5]);
    FG_mobilizer_->set_angle(context_.get(), q[6]);

    NA_mobilizer_->set_angular_rate(context_.get(), v[0]);
    AB_mobilizer_->set_angular_rate(context_.get(), v[1]);
    BC_mobilizer_->set_angular_rate(context_.get(), v[2]);
    CD_mobilizer_->set_angular_rate(context_.get(), v[3]);
    DE_mobilizer_->set_angular_rate(context_.get(), v[4]);
    EF_mobilizer_->set_angular_rate(context_.get(), v[5]);
    FG_mobilizer_->set_angular_rate(context_.get(), v[6]);
  }

  // This method calculates kinematic properties of the end-effector (herein
  // denoted as rigid body G) of a 7-DOF KUKA LBR iiwa robot (14 kg payload).
  // Right-handed orthogonal unit vectors Nx, Ny, Nz are fixed in N (Earth)
  // with Nz vertically upward and right-handed orthogonal unit vectors
  // Gx, Gy, Gz are fixed in G.  The origin of frame N (Earth) is denoted No.
  // The origin Go of end-effector G is located at G's inboard revolute joint.
  //
  // @param[in] q robot's joint angles (generalized coordinates).
  // @param[in] qDt 1st-time-derivatives of q (generalized speeds).
  //
  // @returns  values defined below.
  //
  // std::tuple | Description
  // -----------|-------------------------------------------------
  // X_NG       | Transform with rotation matrix relating Nx, Ny, Nz to
  //            | Gx, Gy, Gz and with Go's position from No, expressed in N.
  // V_NG_N     | G's spatial velocity in N, with G's angular velocity in N
  //            | expressed in N and Go's velocity in N, expressed in N.
  std::tuple<Eigen::Isometry3d, SpatialVelocity<double>>
  CalcEndEffectorPoseAndSpatialVelocity(const double q[7],
                                        const double qDt[7]) {
    SetJointAnglesAndJointRates(q, qDt);

    // For each body, set the pose and spatial velocity in the position,
    // velocity, and acceleration caches with specified values for testing.
    PositionKinematicsCache<double> pc(model_->get_topology());
    VelocityKinematicsCache<double> vc(model_->get_topology());
    // AccelerationKinematicsCache<double> ac(model_->get_topology());

    // Retrieve end-effector pose from position kinematics cache.
    model_->CalcPositionKinematicsCache(*context_, &pc);
    const Eigen::Isometry3d& X_NG = get_body_pose_in_world(pc, *linkG_);

    // Retrieve end-effector spatial velocity from velocity kinematics cache.
    model_->CalcVelocityKinematicsCache(*context_, pc, &vc);
    const SpatialVelocity<double>& V_NG_N =
        get_body_spatial_velocity_in_world(vc, *linkG_);

    // Create tuple to return results.
    return std::make_tuple(Eigen::Isometry3d(X_NG),
                           SpatialVelocity<double>(V_NG_N));
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
  // TODO(mitiguy): When cache entries can be placed in the context, replace by
  // method Body<T>::get_spatial_velocity_in_world(const systems::Context<T>&).
  const SpatialVelocity<double>& get_body_spatial_velocity_in_world(
      const VelocityKinematicsCache<double>& vc,
      const Body<double>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return vc.get_V_WB(topology.get_body(body.get_index()).body_node);
  }

 protected:
  // Utility method to add a FixedOffsetFrame B to a Body bodyA.
  // @param[in] A    Body to which frame B will be attached.
  // @param[in] q123 SpaceXYZ angles describing the rotation matrix relating
  //                 unit vectors Ax, Ay, Az to Bx, By, Bz.
  // @param[in] xyz  Ax, Ay, Az measures of the position from Ao to Bo.
  // @return         New FixedOffsetFrame B that was welded to Body A.
  const FixedOffsetFrame<double>& AddFixedOffsetFrameFromSpaceXYZAnglesAndXYZ(
    const Body<double>& A, const Vector3d& q123, const Vector3d& xyz) {
    const Eigen::Isometry3d X_AB = CreateIsometry3dFromSpaceXYZAnglesAndXYZ(
       q123, xyz);
    return model_->AddFrame<FixedOffsetFrame>(A, X_AB);
  }

  // Utility method to add a revolute joint (mobilizer) between two frames.
  // @param[in] inboard_frame  Mobilizer's inboard joint frame.
  // @param[in] outboard_frame Mobilizer's outboard joint frame.
  // @param[in] revolute_unit_vector  Unit vector orienting the revolute joint.
  // @return New RevoluteMobilizer between inboard_frame and outboard_frame.
  const RevoluteMobilizer<double>& AddRevoluteMobilizer(
     const FixedOffsetFrame<double>& inboard_frame,
     const FixedOffsetFrame<double>& outboard_frame,
     const Eigen::Vector3d& revolute_unit_vector) {
     return model_->AddMobilizer<RevoluteMobilizer>(
        inboard_frame, outboard_frame, revolute_unit_vector);
  }

  // Method to add revolute joint (mobilizer) from Body A to Body B.
  // @param[in] A     Mobilizer's inboard  body (frame AB will be welded to A).
  // @param[in] q123A SpaceXYZ angles describing the rotation matrix relating
  //                  unit vectors Ax, Ay, Az to soon-to-be created frame AB.
  // @param[in] xyzA  Ax, Ay, Az measures of the position from Ao to ABo.
  // @param[in] B     Mobilizer's outboard body (frame BA will be welded to B).
  // @param[in] q123B SpaceXYZ angles describing the rotation matrix relating
  //                  unit vectors Bx, By, Bz to soon-to-be created frame BA.
  // @param[in] xyzB  Bx, By, Bz measures of the position from Bo to BAo.
  // @param[in] revolute_unit_vector  Unit vector orienting the revolute joint.
  // @return          New RevoluteMobilizer between BodyA and BodyB.
  const RevoluteMobilizer<double>& AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
    const Body<double>& A, const Vector3d& q123A, const Vector3d& xyzA,
    const Body<double>& B, const Vector3d& q123B, const Vector3d& xyzB,
    const Eigen::Vector3d& revolute_unit_vector) {
    const FixedOffsetFrame<double>& AB =
      AddFixedOffsetFrameFromSpaceXYZAnglesAndXYZ(A, q123A, xyzA);
    const FixedOffsetFrame<double>& BA =
      AddFixedOffsetFrameFromSpaceXYZAnglesAndXYZ(B, q123B, xyzB);

    return AddRevoluteMobilizer(AB, BA, revolute_unit_vector);
  }

  // Method to add revolute joint (mobilizer) from Body A to Body B.
  // @param[in] A     Mobilizer's inboard  body (frame AB will be welded to A).
  // @param[in] q123A SpaceXYZ angles describing the rotation matrix relating
  //                  unit vectors Ax, Ay, Az to soon-to-be created frame AB.
  // @param[in] xyzA  Ax, Ay, Az measures of the position from Ao to ABo.
  // @param[in] B     Mobilizer's outboard body (frame BA will be welded to B).
  // @param[in] revolute_unit_vector  Unit vector orienting the revolute joint.
  // @return          New RevoluteMobilizer between BodyA and BodyB.
  const RevoluteMobilizer<double>& AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(
    const Body<double>& A, const Vector3d& q123A, const Vector3d& xyzA,
    const Body<double>& B, const Eigen::Vector3d& revolute_unit_vector) {
    return AddRevoluteMobilizerFromSpaceXYZAnglesAndXYZ(A, q123A, xyzA,
              B, Vector3d(0, 0, 0), Vector3d(0, 0, 0), revolute_unit_vector);
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
  MultibodyTreeContext<double>* mbt_context_;
};


}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
