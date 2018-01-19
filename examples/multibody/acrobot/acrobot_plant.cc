#include "drake/examples/multibody/acrobot/acrobot_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace acrobot {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::Cylinder;
using geometry::FrameId;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::SourceId;
using geometry::Sphere;
using drake::multibody::BodyIndex;
using drake::multibody::MultibodyTree;
using drake::multibody::MultibodyTreeContext;
using drake::multibody::MultibodyForces;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using drake::multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::State;

template<typename T>
AcrobotPlant<T>::AcrobotPlant(
    double mass, double length, double gravity) :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::examples::multibody::acrobot::AcrobotPlant>()),
    mass_(mass),
    length_(length),
    gravity_(gravity) {
  BuildMultibodyTreeModel();

  // Some very basic verification that the model is what we expect it to be.
  DRAKE_DEMAND(model_->get_num_positions() == 1);
  DRAKE_DEMAND(model_->get_num_velocities() == 1);
  DRAKE_DEMAND(model_->get_num_states() == 2);

  this->DeclareContinuousState(
      BasicVector<T>(model_->get_num_states()),
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);

  // Declare a vector input of size one for an applied torque about the revolute
  // joint's axis.
  applied_torque_input_ =
      this->DeclareVectorInputPort(BasicVector<T>(1)).get_index();

  // Declare a port that outputs the state.
  state_output_port_ = this->DeclareVectorOutputPort(
      AcrobotState<T>(), &AcrobotPlant::CopyStateOut).get_index();
}

template<typename T>
AcrobotPlant<T>::AcrobotPlant(
    double mass, double length, double gravity,
    geometry::GeometrySystem<double>* geometry_system) :
    AcrobotPlant(mass, length, gravity) {
  DRAKE_DEMAND(geometry_system != nullptr);
  RegisterGeometry(geometry_system);
  DeclareGeometrySystemPorts();
}

template<typename T>
template<typename U>
AcrobotPlant<T>::AcrobotPlant(
    const AcrobotPlant<U> &other) :
    AcrobotPlant(other.mass(), other.length(), other.gravity()) {
  source_id_ = other.source_id_;
  frame_id_ = other.frame_id_;
  // Only declare ports to communicate with a GeometrySystem if the plant is
  // provided with a valid source id.
  if (frame_id_) DeclareGeometrySystemPorts();
}

template<typename T>
void AcrobotPlant<T>::DeclareGeometrySystemPorts() {
  geometry_id_port_ =
      this->DeclareAbstractOutputPort(
          &AcrobotPlant::AllocateFrameIdOutput,
          &AcrobotPlant::CalcFrameIdOutput).get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          &AcrobotPlant::AllocateFramePoseOutput,
          &AcrobotPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
FrameIdVector AcrobotPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  DRAKE_DEMAND(frame_id_ != nullopt);
  FrameIdVector ids(source_id_.value());
  // Add a frame for the one single body in this model.
  ids.AddFrameId(frame_id_.value());
  return ids;
}

template <typename T>
void AcrobotPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector*) const {
  // Just a sanity check.
  DRAKE_DEMAND(source_id_ != nullopt);
  DRAKE_DEMAND(frame_id_ != nullopt);
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
FramePoseVector<T> AcrobotPlant<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  FramePoseVector<T> poses(source_id_.value());
  // There is only one moveable frame.
  poses.mutable_vector().resize(1);
  return poses;
}

template <typename T>
void AcrobotPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_ASSERT(static_cast<int>(poses->vector().size()) == 1);
  DRAKE_ASSERT(model_->get_num_bodies() == 2);

  PositionKinematicsCache<T> pc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);

  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  pose_data[0] = pc.get_X_WB(link_->get_node_index());
}

template <typename T>
void AcrobotPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                    AcrobotState<T>* output) const {
  output->set_theta(joint_->get_angle(context));
  output->set_thetadot(joint_->get_angular_rate(context));
}

template <typename T>
const OutputPort<T>& AcrobotPlant<T>::get_geometry_id_output_port()
const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& AcrobotPlant<T>::get_geometry_pose_output_port()
const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
const InputPortDescriptor<T>& AcrobotPlant<T>::get_input_port() const {
  return systems::System<T>::get_input_port(applied_torque_input_);
}

template <typename T>
const OutputPort<T>& AcrobotPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_output_port_);
}

template<typename T>
void AcrobotPlant<T>::BuildMultibodyTreeModel() {
  model_ = std::make_unique<MultibodyTree<T>>();
  DRAKE_DEMAND(model_ != nullptr);

  // Position of the com of the acrobot's body (in this case a point mass) in
  // the body's frame. The body's frame's origin Bo is defined to be at the
  // world's origin Wo, through which the rotation axis passes.
  // With the acrobot at rest pointing in the downwards -z direction, the body
  // frame B and the world frame W are coincident.
  const Vector3<double> p_BoBcm_B = -length() * Vector3<double>::UnitZ();

  // Define each link's spatial inertia about the body's origin Bo.
  UnitInertia<double> G_Bo =
      UnitInertia<double>::PointMass(p_BoBcm_B);
  SpatialInertia<double> M_Bo(mass(), p_BoBcm_B, G_Bo);

  link_ = &model_->template AddBody<RigidBody>(M_Bo);

  joint_ = &model_->template AddJoint<RevoluteJoint>(
      "Joint",
      model_->get_world_body(), {}, /* frame F IS the the world frame W. */
      *link_, {}, /* frame M IS the the body frame B. */
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

//  model_->template AddJointActuator("JointMotor", *joint_);

  model_->template AddForceElement<UniformGravityFieldElement>(
      -gravity() * Vector3d::UnitZ());

  model_->Finalize();
}

template<typename T>
void AcrobotPlant<T>::RegisterGeometry(
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);

  source_id_ = geometry_system->RegisterSource("multibody_acrobot");

  // The acrobot's body frame B is defined to have its origin Bo coincident
  // with the world's origin Wo. For the acrobot at rest pointing downwards in
  // the -z direction, the acrobot's body frame B is coincident with the world
  // frame W.
  frame_id_ = geometry_system->RegisterFrame(
      source_id_.value(),
      GeometryFrame("AcrobotFrame", Isometry3<double>::Identity()));

  // Pose of the acrobot's point mass in the body's frame.
  const Vector3<double> p_BoBcm_B = -length() * Vector3<double>::UnitZ();

  // Pose of the geometry frame G in the body frame B.
  const Isometry3<double> X_BG{Translation3<double>(p_BoBcm_B)};

  // A sphere at the world's origin.
  geometry_system->RegisterGeometry(
      source_id_.value(), frame_id_.value(),
      std::make_unique<GeometryInstance>(
          Isometry3d::Identity(), /* Geometry pose in link's frame */
          std::make_unique<Sphere>(length() / 8)));

  // A rod (massless in the multibody model) between the revolute pin and the
  // point mass.
  geometry_system->RegisterGeometry(
      source_id_.value(), frame_id_.value(),
      std::make_unique<GeometryInstance>(
          Isometry3d{Translation3d(p_BoBcm_B / 2.0)},
          std::make_unique<Cylinder>(length() / 15, length())));

  // A sphere at the point mass:
  geometry_system->RegisterGeometry(
      source_id_.value(), frame_id_.value(),
      std::make_unique<GeometryInstance>(
          X_BG, /* Geometry pose in link's frame */
          std::make_unique<Sphere>(length() / 5)));
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
AcrobotPlant<T>::DoMakeContext() const {
  return std::make_unique<MultibodyTreeContext<T>>(model_->get_topology());
}

template<typename T>
void AcrobotPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = model_->get_num_velocities();

  // Mass matrix:
  MatrixX<T> M(nv, nv);
  // Forces:
  MultibodyForces<T> forces(*model_);
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->get_num_bodies());
  // Generalized accelerations:
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  // TODO(amcastro-tri): Eval() these from the context.
  PositionKinematicsCache<T> pc(model_->get_topology());
  VelocityKinematicsCache<T> vc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);
  model_->CalcVelocityKinematicsCache(context, pc, &vc);

  // Compute forces applied through force elements. This effectively resets
  // the forces to zero and adds in contributions due to force elements:
  model_->CalcForceElementsContribution(context, pc, vc, &forces);

  // Add in input forces:
  joint_->AddInTorque(context, get_tau(context), &forces);

  model_->CalcMassMatrixViaInverseDynamics(context, &M);

  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both Fapplied_Bo_W_array and tau_applied
  // get overwritten on output. This is not important in this case since we
  // don't need their values anymore. Please see the documentation for
  // CalcInverseDynamics() for details.
  // With vdot = 0, this computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces.mutable_body_forces();
  VectorX<T>& tau_array = forces.mutable_generalized_forces();
  model_->CalcInverseDynamics(
      context, pc, vc, vdot,
      F_BBo_W_array, tau_array,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau_array);

  vdot = M.ldlt().solve(-tau_array);

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(model_->get_num_states());
  // For this simple model v = qdot.
  xdot << v, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
void AcrobotPlant<T>::SetAngle(
    Context<T>* context, const T& angle) const {
  DRAKE_DEMAND(context != nullptr);
  joint_->set_angle(context, angle);
}

}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::examples::multibody::acrobot::AcrobotPlant)
