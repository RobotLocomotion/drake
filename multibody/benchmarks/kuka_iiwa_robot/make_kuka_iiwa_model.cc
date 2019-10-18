#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"

#include "drake/common/default_scalars.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

using drake::multibody::FixedOffsetFrame;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;

using std::make_unique;
using std::unique_ptr;

namespace internal {

template <typename T>
const RevoluteJoint<T>&
KukaIiwaModelBuilder<T>::AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
    const std::string& joint_name,
    const Body<T>& A,
    const Vector3<double>& q123A, const Vector3<double>& xyzA,
    const Body<T>& B, const Vector3<double>& revolute_unit_vector,
    multibody::internal::MultibodyTree<T>* model) {
  // Create transform from inboard body A to mobilizer inboard frame Ab.
  const math::RollPitchYaw<double> rpy(q123A);
  const math::RotationMatrix<double> R_AAb(rpy);
  const math::RigidTransformd X_AAb(R_AAb, xyzA);

  // Create transform from outboard body B to mobilizer outboard frame Ba.
  const math::RigidTransformd X_BBa;  // Identity transform.

  return model->template AddJoint<RevoluteJoint>(joint_name,
                              A, X_AAb, B, X_BBa, revolute_unit_vector);
}

template <typename T>
unique_ptr<multibody::internal::MultibodyTree<T>>
KukaIiwaModelBuilder<T>::Build() const {
  // Create a mostly empty MultibodyTree (it has a built-in "world" body).
  // Newtonian reference frame (linkN) is the world body.
  auto model = make_unique<multibody::internal::MultibodyTree<T>>();

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
  const RigidBody<T>& linkA = model->AddRigidBody("iiwa_link_1", M_AAo_A);
  const RigidBody<T>& linkB = model->AddRigidBody("iiwa_link_2", M_BBo_B);
  const RigidBody<T>& linkC = model->AddRigidBody("iiwa_link_3", M_CCo_C);
  const RigidBody<T>& linkD = model->AddRigidBody("iiwa_link_4", M_DDo_D);
  const RigidBody<T>& linkE = model->AddRigidBody("iiwa_link_5", M_EEo_E);
  const RigidBody<T>& linkF = model->AddRigidBody("iiwa_link_6", M_FFo_F);
  const RigidBody<T>& linkG = model->AddRigidBody("iiwa_link_7", M_GGo_G);

  // Create a revolute joint between linkN (Newtonian frame/world) and linkA
  // using two joint-frames, namely "Na" and "An".  The "inboard frame" Na is
  // welded to linkN and the "outboard frame" An is welded to linkA.
  // The orientation and position of Na relative to linkN are specified by the
  // second and third arguments in the following method, namely with SpaceXYZ
  // angles and a position vector. Alternately, frame An is regarded as
  // coincident with linkA.
  const Joint<T>* joint{nullptr};
  const Body<T>& linkN = model->world_body();
  joint = &AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      "iiwa_joint_1",
      linkN, joint_1_rpy_, joint_1_xyz_,
      linkA, Eigen::Vector3d::UnitZ(), model.get());
  model->AddJointActuator("iiwa_actuator_1", *joint);

  // Create a revolute joint between linkA and linkB.
  joint = &AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      "iiwa_joint_2",
      linkA, joint_2_rpy_, joint_2_xyz_,
      linkB, Eigen::Vector3d::UnitZ(), model.get());
  model->AddJointActuator("iiwa_actuator_2", *joint);

  // Create a revolute joint between linkB and linkC.
  joint = &AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      "iiwa_joint_3",
      linkB, joint_3_rpy_, joint_3_xyz_,
      linkC, Eigen::Vector3d::UnitZ(), model.get());
  model->AddJointActuator("iiwa_actuator_3", *joint);

  // Create a revolute joint between linkB and linkC.
  joint = &AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      "iiwa_joint_4",
      linkC, joint_4_rpy_, joint_4_xyz_,
      linkD, Eigen::Vector3d::UnitZ(), model.get());
  model->AddJointActuator("iiwa_actuator_4", *joint);

  // Create a revolute joint between linkD and linkE.
  joint = &AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      "iiwa_joint_5",
      linkD, joint_5_rpy_, joint_5_xyz_,
      linkE, Eigen::Vector3d::UnitZ(), model.get());
  model->AddJointActuator("iiwa_actuator_5", *joint);

  // Create a revolute joint between linkE and linkF.
  joint = &AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      "iiwa_joint_6",
      linkE, joint_6_rpy_, joint_6_xyz_,
      linkF, Eigen::Vector3d::UnitZ(), model.get());
  model->AddJointActuator("iiwa_actuator_6", *joint);

  // Create a revolute joint between linkE and linkF.
  joint = &AddRevoluteJointFromSpaceXYZAnglesAndXYZ(
      "iiwa_joint_7",
      linkF, joint_7_rpy_, joint_7_xyz_,
      linkG, Eigen::Vector3d::UnitZ(), model.get());
  model->AddJointActuator("iiwa_actuator_7", *joint);

  // Add arbitrary tool frame.
  model->template AddFrame<FixedOffsetFrame>(
      "tool_arbitrary", model->GetFrameByName("iiwa_link_7"),
      math::RigidTransformd(Eigen::Vector3d(0.1, 0.2, 0.3)));

  // Add force element for a constant gravity pointing downwards, that is, in
  // the negative z-axis direction.
  const Eigen::Vector3d gravity_vector = -gravity_ * Eigen::Vector3d::UnitZ();
  model->mutable_gravity_field().set_gravity_vector(gravity_vector);

  // Finalize() stage sets the topology (model is built).
  if (finalize_model_) model->Finalize();
  return model;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class internal::KukaIiwaModelBuilder);

}  // namespace internal
}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
