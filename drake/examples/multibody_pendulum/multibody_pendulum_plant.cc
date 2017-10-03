#include "drake/examples/multibody_pendulum/multibody_pendulum_plant.h"

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
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace multibody_pendulum {

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
using multibody::BodyIndex;
using multibody::MultibodyTree;
using multibody::PositionKinematicsCache;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::SpatialAcceleration;
using multibody::SpatialForce;
using multibody::SpatialInertia;
using multibody::UniformGravityFieldElement;
using multibody::UnitInertia;
using multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;
using systems::OutputPort;
using systems::State;

template<typename T>
MultibodyPendulumPlant<T>::MultibodyPendulumPlant(
    double mass, double length, double gravity) :
    MultibodyPendulumPlant(mass, length, gravity, SourceId{}, FrameId{}) {}

template<typename T>
MultibodyPendulumPlant<T>::MultibodyPendulumPlant(
    double mass, double length, double gravity,
    SourceId source_id, FrameId frame_id) :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::examples::multibody_pendulum::MultibodyPendulumPlant>()),
    mass_(mass), length_(length), gravity_(gravity),
    source_id_(source_id), frame_id_(frame_id) {
  // Build the MultibodyTree model for this plant.
  BuildMultibodyTreeModel();

  // Some very basic verification tha the model is what we expect it to be.
  DRAKE_DEMAND(model_->get_num_positions() == 1);
  DRAKE_DEMAND(model_->get_num_velocities() == 1);
  DRAKE_DEMAND(model_->get_num_states() == 2);

  this->DeclareContinuousState(
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);

  if (frame_id.is_valid()) {
    geometry_id_port_ =
        this->DeclareAbstractOutputPort(
            &MultibodyPendulumPlant::AllocateFrameIdOutput,
            &MultibodyPendulumPlant::CalcFrameIdOutput).get_index();
    geometry_pose_port_ =
        this->DeclareAbstractOutputPort(
            &MultibodyPendulumPlant::AllocateFramePoseOutput,
            &MultibodyPendulumPlant::CalcFramePoseOutput).get_index();
  }
}

template<typename T>
MultibodyPendulumPlant<T>::MultibodyPendulumPlant(
    double mass, double length, double gravity,
    geometry::GeometrySystem<double>* geometry_system) :
    MultibodyPendulumPlant(mass, length, gravity) {
  DRAKE_DEMAND(geometry_system != nullptr);

  RegisterGeometry(geometry_system);

  geometry_id_port_ =
      this->DeclareAbstractOutputPort(
          &MultibodyPendulumPlant::AllocateFrameIdOutput,
          &MultibodyPendulumPlant::CalcFrameIdOutput).get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          &MultibodyPendulumPlant::AllocateFramePoseOutput,
          &MultibodyPendulumPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
FrameIdVector MultibodyPendulumPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_.is_valid());
  DRAKE_DEMAND(frame_id_.is_valid());
  FrameIdVector ids(source_id_);
  // Add a frame for the one single body in this model.
  ids.AddFrameId(frame_id_);
  return ids;
}

template <typename T>
void MultibodyPendulumPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector*) const {
  // Just a sanity check.
  DRAKE_DEMAND(source_id_.is_valid());
  DRAKE_DEMAND(frame_id_.is_valid());
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
FramePoseVector<T> MultibodyPendulumPlant<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  DRAKE_ABORT_MSG("There is no implementation for T != double.");
}

template <>
FramePoseVector<double> MultibodyPendulumPlant<double>::AllocateFramePoseOutput(
    const Context<double>&) const {
  DRAKE_DEMAND(source_id_.is_valid());
  FramePoseVector<double> poses(source_id_);
  // There is only one moveable frame.
  poses.mutable_vector().resize(1);
  return poses;
}

template <typename T>
void MultibodyPendulumPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_ABORT_MSG("There is no implementation for T != double.");
}

template <>
void MultibodyPendulumPlant<double>::CalcFramePoseOutput(
    const Context<double>& context, FramePoseVector<double>* poses) const {
  DRAKE_ASSERT(static_cast<int>(poses->vector().size()) == 1);
  DRAKE_ASSERT(model_->get_num_bodies() == 2);

  PositionKinematicsCache<double> pc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);

  std::vector<Isometry3<double>>& pose_data = poses->mutable_vector();
  pose_data[0] = pc.get_X_WB(link_->get_node_index());
}

template <typename T>
const OutputPort<T>& MultibodyPendulumPlant<T>::get_geometry_id_output_port()
const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& MultibodyPendulumPlant<T>::get_geometry_pose_output_port()
const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template<typename T>
template<typename U>
MultibodyPendulumPlant<T>::MultibodyPendulumPlant(
    const MultibodyPendulumPlant<U>& other) :
    MultibodyPendulumPlant<T>(
        other.get_mass(),
        other.get_length(),
        other.get_gravity(),
        other.source_id_, other.frame_id_) {}

template<typename T>
void MultibodyPendulumPlant<T>::BuildMultibodyTreeModel() {
  model_ = std::make_unique<MultibodyTree<T>>();
  DRAKE_DEMAND(model_ != nullptr);

  // Pose of the com of the pendulum's body (in this case a point mass) in the
  // body's frame. The body's frame's origin Bo is defined to be at the world's
  // origin, where the rotation axis is.
  const Vector3<double> p_BoBcm_B = -get_length() * Vector3<double>::UnitZ();

  // Define each link's spatial inertia about the body's origin Bo.
  UnitInertia<double> G_Bo =
      UnitInertia<double>::PointMass(p_BoBcm_B);

  SpatialInertia<double> M_Bo(get_mass(), p_BoBcm_B, G_Bo);

  link_ = &model_->template AddBody<RigidBody>(M_Bo);

  joint_ = &model_->template AddJoint<RevoluteJoint>(
      "Joint",
      model_->get_world_body(), {}, /* frame F IS the the world frame W. */
      *link_, {}, /* frame M IS the the body frame B. */
      Vector3d::UnitY()); /* pendulum oscillates in the x-z plane. */

  model_->template AddForceElement<UniformGravityFieldElement>(
      -get_gravity() * Vector3d::UnitZ());

  model_->Finalize();
}

template<typename T>
void MultibodyPendulumPlant<T>::RegisterGeometry(
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);

  source_id_ = geometry_system->RegisterSource("multibody_pendulum");

  frame_id_ = geometry_system->RegisterFrame(
      source_id_,
      GeometryFrame("PendulumFrame", Isometry3<double>::Identity()));

  // Pose of the pendulum's point mass in the body's frame.
  const Vector3<double> p_BoBcm_B = -get_length() * Vector3<double>::UnitZ();
  const Isometry3<double> X_BG{Translation3<double>(p_BoBcm_B)};

  // A sphere at the world's origin.
  geometry_system->RegisterGeometry(
      source_id_, frame_id_,
      std::make_unique<GeometryInstance>(
          Isometry3d::Identity(), /* Geometry pose in link's frame */
          std::make_unique<Sphere>(get_length() / 8)));

  // A rod (massless in the multibody model) between the revolute pin and the
  // point mass.
  geometry_system->RegisterGeometry(
      source_id_, frame_id_,
      std::make_unique<GeometryInstance>(
          Isometry3d{Translation3d(p_BoBcm_B / 2.0)},
          std::make_unique<Cylinder>(get_length() / 15, get_length())));

  // A sphere at the point mass:
  geometry_system->RegisterGeometry(
      source_id_, frame_id_,
      std::make_unique<GeometryInstance>(
          X_BG, /* Geometry pose in link's frame */
          std::make_unique<Sphere>(get_length() / 5)));
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyPendulumPlant<T>::DoMakeContext() const {
  return model_->CreateDefaultContext();
}

template<typename T>
void MultibodyPendulumPlant<T>::DoCalcTimeDerivatives(
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

  // Compute applied forces.
  std::vector<SpatialForce<T>> Fapplied_Bo_W_array(model_->get_num_bodies());
  VectorX<T> tau_applied(model_->get_num_velocities());
  model_->CalcForceElementsContribution(
      context, pc, vc, &Fapplied_Bo_W_array, &tau_applied);
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->get_num_bodies());

  VectorX<T> vdot = VectorX<T>::Zero(nv);
  VectorX<T> C(nv);
  model_->CalcInverseDynamics(
      context, pc, vc, vdot, Fapplied_Bo_W_array, tau_applied,
      &A_WB_array, &Fapplied_Bo_W_array, &C);

  auto v = x.bottomRows(nv);

  VectorX<T> xdot(model_->get_num_states());
  xdot << v, M.llt().solve(-C);
  derivatives->SetFromVector(xdot);
}

template<typename T>
void MultibodyPendulumPlant<T>::SetAngle(
    Context<T>* context, const T& angle) const {
  joint_->set_angle(context, angle);
}

}  // namespace multibody_pendulum
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::multibody_pendulum::MultibodyPendulumPlant)
