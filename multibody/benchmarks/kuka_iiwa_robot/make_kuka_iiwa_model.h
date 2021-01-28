#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {
namespace internal {

template <typename T>
class KukaIiwaModelBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KukaIiwaModelBuilder);

  template <typename U>
  using MultibodyTree = multibody::internal::MultibodyTree<U>;

  // Instantiate a builder to make a MultibodyTree model of the KUKA iiwa arm
  // as specified in this class' documentation.
  // The world z-unit vector is vertically upward.
  // @param[in] finalize_model
  //   If `true`, the model is finalized with MultibodyTree::Finalize().
  //   A non-finalized model can be requested if adding more multibody elements
  //   is desired.
  // @param[in] gravity
  //   The model's acceleration of gravity. `gravity > 0` means the gravity
  //   field is directed opposite the world upward z-unit vector (i.e.
  //   downwards).
  KukaIiwaModelBuilder(bool finalize_model, double gravity) :
      gravity_(gravity),
      finalize_model_(finalize_model) {}

  // Construct a 7-DOF Kuka iiwa robot arm (from file kuka_iiwa_robot.urdf).
  // The robot is constructed with 7 revolute joints.
  // The world z-unit vector is vertically upward. The acceleration of gravity
  // is of 9.81 m/s² directed opposite the world upward z-unit vector.
  // TODO(amcastro-tri): consider adding an input for the pose of the base in
  // the world.
  std::unique_ptr<MultibodyTree<T>> Build() const;

 private:
  // Helper method to add revolute joint from Body A to Body B.
  // @param[in] A
  //   Joint's parent  body (frame Ab will be welded to A).
  // @param[in] q123A
  //   SpaceXYZ angles describing the rotation matrix relating unit vectors
  //   Ax, Ay, Az to soon-to-be created frame Ab.
  //   The SpaceXYZ (extrinsic, rotations about space fixed axes) convention is
  //   equivalent to the BodyZYX (intrinsic, rotations about body fixed axes)
  //   convention. This rotation convention is exactly the one adopted in URDF
  //   files with roll (about space fixed X), pitch (about space fixed Y) and
  //   yaw (about space fixed Z).
  // @param[in] xyzA
  //   Ax, Ay, Az measures of the position from Ao to Abo.
  // @param[in] B
  //   Joint's child body (frame Ba will be welded to B so it is coincident with
  //   body B's frame). In other words, joint's frame Ba on the child body will
  //   be coincident with the outboard body frame B.
  // @param[in] revolute_unit_vector
  //   Unit vector expressed in frame Ab that characterizes a positive rotation
  //   of Ba from Ab (right-hand-rule).
  // @param[out] model
  //   The MultibodyTree to which the new joint will be added.
  // @return RevoluteJoint from frame Ab on Body A to frame Ba on Body B.
  static const RevoluteJoint<T>& AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      const std::string& joint_name,
      const Body<T>& A,
      const Vector3<double>& q123A, const Vector3<double>& xyzA,
      const Body<T>& B, const Vector3<double>& revolute_unit_vector,
      MultibodyTree<T>* model);

  // Model's parameters:

  // Mass of each link (in kg).
  double massA_ = 5.76;
  double massB_ = 6.35;
  double massC_ = 3.5;
  double massD_ = 3.5;
  double massE_ = 3.5;
  double massF_ = 1.8;
  double massG_ = 1.2;

  // Position of each body's center of mass from body origin, expressed in body.
  // Example: For a body B with center of mass Bcm and origin Bo, p_BoBcm_B is
  // the position from Bo to Bcm, expressed in terms of Bx, By, Bz (in meters).
  Eigen::Vector3d p_AoAcm_A_{0,     -0.03,   0.12};
  Eigen::Vector3d p_BoBcm_B_{0.0003, 0.059,  0.042};
  Eigen::Vector3d p_CoCcm_C_{0,      0.03,   0.13};
  Eigen::Vector3d p_DoDcm_D_{0,      0.067,  0.034};
  Eigen::Vector3d p_EoEcm_E_{0.0001, 0.021,  0.076};
  Eigen::Vector3d p_FoFcm_F_{0,      0.0006, 0.0004};
  Eigen::Vector3d p_GoGcm_G_{0,      0,      0.02};

  // Inertia matrix of each body about its center of mass, expressed in body.
  // Example: For a body B with center of mass Bcm, I_Bcm_B is B's inertia
  // matrix about Bcm, expressed in terms of Bx, By, Bz (in kg * meters^2).
  RotationalInertia<double> I_AAcm_A_{0.033,  0.0333, 0.0123};
  RotationalInertia<double> I_BBcm_B_{0.0305, 0.0304, 0.011};
  RotationalInertia<double> I_CCcm_C_{0.025,  0.0238, 0.0076};
  RotationalInertia<double> I_DDcm_D_{0.017,  0.0164, 0.006};
  RotationalInertia<double> I_EEcm_E_{0.01,   0.0087, 0.00449};
  RotationalInertia<double> I_FFcm_F_{0.0049, 0.0047, 0.0036};
  RotationalInertia<double> I_GGcm_G_{0.001,  0.001,  0.001};

  // These parameters define the pose X_PF of each joint's inboard frame F in
  // the frame of the inboard body P.
  // joint_i_rpy_ provides the SpaceXYZ angles (aka roll, pitch, and yaw in the
  // URDF file format world) describing orientation R_PF of the i-th joint's
  // inboard frame F in the parent body frame P.
  // joint_1_xyz_ provides the position p_PoFo_P of the i-th joint's inboard
  // frame origin Fo measured and expressed in the parent body frame P.
  Eigen::Vector3d
      joint_1_rpy_{0, 0, 0}, joint_1_xyz_{0, 0, 0.1575};
  Eigen::Vector3d
      joint_2_rpy_{M_PI_2, 0, M_PI}, joint_2_xyz_{0, 0, 0.2025};
  Eigen::Vector3d
      joint_3_rpy_{M_PI_2, 0, M_PI}, joint_3_xyz_{0, 0.2045, 0};
  Eigen::Vector3d
      joint_4_rpy_{M_PI_2, 0, 0}, joint_4_xyz_{0, 0, 0.2155};
  Eigen::Vector3d
      joint_5_rpy_{-M_PI_2, M_PI, 0}, joint_5_xyz_{0, 0.1845, 0};
  Eigen::Vector3d
      joint_6_rpy_{M_PI_2, 0, 0}, joint_6_xyz_{0, 0, 0.2155};
  Eigen::Vector3d
      joint_7_rpy_{-M_PI_2, M_PI, 0}, joint_7_xyz_{0, 0.081, 0};

  // Earth's default gravitational acceleration, in m/s².
  double gravity_{9.81};

  // Flag indicating if MultibodyTree::Finalize() will be called on the model.
  bool finalize_model_;
};

}  // namespace internal

/// This method makes a MultibodyTree model for a Kuka Iiwa arm as specified
/// in the file kuka_iiwa_robot.urdf contained in this same directory.
/// Links can be accessed by their name "iiwa_link_1" (base) through
/// "iiwa_link_7" (end effector). The "world" body can be accessed with
/// MultibodyTree::world_body().
/// Joints can be accessed by their name "iiwa_joint_1" (from the base) through
/// "iiwa_joint_7" (to the end effector).
/// The new MultibodyTree model is finalized by MultibodyTree::Finalize() and
/// therefore no more modeling elements can be added.
/// @param[in] finalize_model
///   If `true`, the model is finalized with MultibodyTree::Finalize().
///   A non-finalized model can be requested if adding more multibody elements
///   is desired.
/// @param[in] gravity
///   The value of the acceleration of gravity, in m/s².
template <typename T>
std::unique_ptr<multibody::internal::MultibodyTree<T>> MakeKukaIiwaModel(
    bool finalize_model = true, double gravity = 9.81) {
  internal::KukaIiwaModelBuilder<T> builder(finalize_model, gravity);
  return builder.Build();
}

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
