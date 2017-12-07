#include "drake/examples/multibody/pendulum/pendulum_plant.h"

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
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace pendulum {

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
using systems::OutputPort;
using systems::State;

template<typename T>
PendulumPlant<T>::PendulumPlant(
    double mass, double length, double gravity) :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::examples::multibody::pendulum::PendulumPlant>()),
    mass_(mass),
    length_(length),
    gravity_(gravity) {
  BuildMultibodyTreeModel();

  // Some very basic verification that the model is what we expect it to be.
  DRAKE_DEMAND(model_->get_num_positions() == 1);
  DRAKE_DEMAND(model_->get_num_velocities() == 1);
  DRAKE_DEMAND(model_->get_num_states() == 2);

  this->DeclareContinuousState(
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);
}

template<typename T>
PendulumPlant<T>::PendulumPlant(
    double mass, double length, double gravity,
    geometry::GeometrySystem<double>* geometry_system) :
    PendulumPlant(mass, length, gravity) {
  DRAKE_DEMAND(geometry_system != nullptr);
  RegisterGeometry(geometry_system);
  DeclareGeometrySystemPorts();
}

template<typename T>
template<typename U>
PendulumPlant<T>::PendulumPlant(
    const PendulumPlant<U> &other) :
    PendulumPlant(other.mass(), other.length(), other.gravity()) {
  source_id_ = other.source_id_;
  frame_id_ = other.frame_id_;
  // Only declare ports to communicate with a GeometrySystem if the plant is
  // provided with a valid source id.
  if (frame_id_) DeclareGeometrySystemPorts();
}

template<typename T>
void PendulumPlant<T>::DeclareGeometrySystemPorts() {
  geometry_id_port_ =
      this->DeclareAbstractOutputPort(
          &PendulumPlant::AllocateFrameIdOutput,
          &PendulumPlant::CalcFrameIdOutput).get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          &PendulumPlant::AllocateFramePoseOutput,
          &PendulumPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
FrameIdVector PendulumPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  DRAKE_DEMAND(frame_id_ != nullopt);
  FrameIdVector ids(source_id_.value());
  // Add a frame for the one single body in this model.
  ids.AddFrameId(frame_id_.value());
  return ids;
}

template <typename T>
void PendulumPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector*) const {
  // Just a sanity check.
  DRAKE_DEMAND(source_id_ != nullopt);
  DRAKE_DEMAND(frame_id_ != nullopt);
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
FramePoseVector<T> PendulumPlant<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  FramePoseVector<T> poses(source_id_.value());
  // There is only one moveable frame.
  poses.mutable_vector().resize(1);
  return poses;
}

template <typename T>
void PendulumPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_ASSERT(static_cast<int>(poses->vector().size()) == 1);
  DRAKE_ASSERT(model_->get_num_bodies() == 2);

  PositionKinematicsCache<T> pc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);

  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  pose_data[0] = pc.get_X_WB(link_->get_node_index());
}

template <typename T>
const OutputPort<T>& PendulumPlant<T>::get_geometry_id_output_port()
const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& PendulumPlant<T>::get_geometry_pose_output_port()
const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template<typename T>
void PendulumPlant<T>::BuildMultibodyTreeModel() {
  model_ = std::make_unique<MultibodyTree<T>>();
  DRAKE_DEMAND(model_ != nullptr);

  // Position of the com of the pendulum's body (in this case a point mass) in
  // the body's frame. The body's frame's origin Bo is defined to be at the
  // world's origin Wo, through which the rotation axis passes.
  // With the pendulum at rest pointing in the downwards -z direction, the body
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
      Vector3d::UnitY()); /* pendulum oscillates in the x-z plane. */

  model_->template AddForceElement<UniformGravityFieldElement>(
      -gravity() * Vector3d::UnitZ());

  model_->Finalize();
}

template<typename T>
void PendulumPlant<T>::RegisterGeometry(
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);

  source_id_ = geometry_system->RegisterSource("multibody_pendulum");

  // The pendulum's body frame B is defined to have its origin Bo coincident
  // with the world's origin Wo. For the pendulum at rest pointing downwards in
  // the -z direction, the pendulum's body frame B is coincident with the world
  // frame W.
  frame_id_ = geometry_system->RegisterFrame(
      source_id_.value(),
      GeometryFrame("PendulumFrame", Isometry3<double>::Identity()));

  // Pose of the pendulum's point mass in the body's frame.
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
PendulumPlant<T>::DoMakeContext() const {
  return model_->CreateDefaultContext();
}

template<typename T>
void PendulumPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = model_->get_num_velocities();

  PositionKinematicsCache<T> pc(model_->get_topology());
  VelocityKinematicsCache<T> vc(model_->get_topology());

  MatrixX<T> M(nv, nv);
  model_->CalcMassMatrixViaInverseDynamics(context, &M);

  model_->CalcPositionKinematicsCache(context, &pc);
  model_->CalcVelocityKinematicsCache(context, pc, &vc);

  // Compute applied forces, which in this case will contain gravity.
  std::vector<SpatialForce<T>> Fapplied_Bo_W_array(model_->get_num_bodies());
  VectorX<T> tau_applied(model_->get_num_velocities());
  model_->CalcForceElementsContribution(
      context, pc, vc, &Fapplied_Bo_W_array, &tau_applied);
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->get_num_bodies());

  VectorX<T> vdot = VectorX<T>::Zero(nv);
  VectorX<T> C(nv);
  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both Fapplied_Bo_W_array and tau_applied
  // get overwritten on output. This is not important in this case since we
  // don't need their values anymore. Please see the documentation for
  // CalcInverseDynamics() for details.
  model_->CalcInverseDynamics(
      context, pc, vc, vdot, Fapplied_Bo_W_array, tau_applied,
      &A_WB_array, &Fapplied_Bo_W_array, &C);

  auto v = x.bottomRows(nv);

  VectorX<T> xdot(model_->get_num_states());
  // For this simple model v = qdot.
  xdot << v, M.llt().solve(-C);
  derivatives->SetFromVector(xdot);
}

template<typename T>
void PendulumPlant<T>::SetAngle(
    Context<T>* context, const T& angle) const {
  DRAKE_DEMAND(context != nullptr);
  joint_->set_angle(context, angle);
}

}  // namespace pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::examples::multibody::pendulum::PendulumPlant)
