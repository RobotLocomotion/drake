#include "drake/examples/n_link_pendulum/n_link_pendulum_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace examples {
namespace n_link_pendulum {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::FrameId;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::SourceId;
using geometry::Sphere;
using multibody::BodyIndex;
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
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    double mass, double length, double radius, int num_links) :
    NLinkPendulumPlant(
        mass, length, radius, num_links,
        SourceId{}, std::vector<geometry::FrameId>{}) {}

template<typename T>
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    double mass, double length, double radius, int num_links,
    geometry::SourceId source_id,
    const std::vector<geometry::FrameId>& body_index_to_frame_id_map) :
    mass_(mass), length_(length), radius_(radius), num_links_(num_links),
    source_id_(source_id),
    body_index_to_frame_id_map_(body_index_to_frame_id_map) {

  // Build the MultibodyTree model for this plant.
  BuildMultibodyTreeModel();

  // Some very basic verification tha the model is what we expect it to be.
  DRAKE_DEMAND(model_.get_num_positions() == num_links);
  DRAKE_DEMAND(model_.get_num_velocities() == num_links);
  DRAKE_DEMAND(model_.get_num_states() == 2 * num_links);

  this->DeclareContinuousState(
      model_.get_num_positions(),
      model_.get_num_velocities(), 0 /* num_z */);

  if (body_index_to_frame_id_map.size() !=0 ) {
    geometry_id_port_ =
        this->DeclareAbstractOutputPort(
            &NLinkPendulumPlant::AllocateFrameIdOutput,
            &NLinkPendulumPlant::CalcFrameIdOutput).get_index();
    geometry_pose_port_ =
        this->DeclareAbstractOutputPort(
            &NLinkPendulumPlant::AllocateFramePoseOutput,
            &NLinkPendulumPlant::CalcFramePoseOutput).get_index();
  }
}

template<typename T>
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    double mass, double length, double radius, int num_links,
    geometry::GeometrySystem<T>* geometry_system) :
    NLinkPendulumPlant(mass, length, radius, num_links) {
  DRAKE_DEMAND(geometry_system != nullptr);

  RegisterGeometry(geometry_system);

  geometry_id_port_ =
      this->DeclareAbstractOutputPort(
          &NLinkPendulumPlant::AllocateFrameIdOutput,
          &NLinkPendulumPlant::CalcFrameIdOutput).get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          &NLinkPendulumPlant::AllocateFramePoseOutput,
          &NLinkPendulumPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
FrameIdVector NLinkPendulumPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_.is_valid());
  DRAKE_DEMAND(
      static_cast<int>(body_index_to_frame_id_map_.size()) == get_num_links());
  FrameIdVector ids(source_id_);
  // Add a frame for each Body in the model. Skip the world.
  for (BodyIndex body_index(1);
       body_index < model_.get_num_bodies(); ++body_index) {
    ids.AddFrameId(body_index_to_frame_id_map_[body_index]);
  }
  return ids;
}

template <typename T>
void NLinkPendulumPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector* ids) const {
  DRAKE_DEMAND(ids->size() == get_num_links());
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
FramePoseVector<T> NLinkPendulumPlant<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_.is_valid());
  FramePoseVector<T> poses(source_id_);
  poses.mutable_vector().resize(get_num_links());
  return poses;
}

template <typename T>
void NLinkPendulumPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_ASSERT(static_cast<int>(poses->vector().size()) == get_num_links());

  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);

  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  for (int frame_index{0}; frame_index < get_num_links(); ++frame_index) {
    const BodyIndex body_index(frame_index + 1);
    pose_data[frame_index] =
        pc.get_X_WB(model_.get_body(body_index).get_node_index());
  }
}

template <typename T>
const OutputPort<T>& NLinkPendulumPlant<T>::get_geometry_id_output_port()
const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& NLinkPendulumPlant<T>::get_geometry_pose_output_port()
const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template<typename T>
template<typename U>
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    const NLinkPendulumPlant<U>& other) :
    NLinkPendulumPlant<T>(
        other.get_mass(),
        other.get_length(),
        other.get_radius(),
        other.get_num_links(),
        other.source_id(), other.body_index_to_frame_id_map_) {}

template<typename T>
void NLinkPendulumPlant<T>::BuildMultibodyTreeModel() {

  // Evenly distribute mass across the n-links.
  double link_mass = get_mass() / get_num_links();
  double link_length = get_length() / get_num_links();

  // Define each link's spatial inertia about its center of mass Bcm.
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::SolidCylinder(
          get_radius(), link_length, Vector3<double>::UnitY());
  SpatialInertia<double> M_Bcm(link_mass, Vector3<double>::Zero(), G_Bcm);

  // A pointer to the last added link.
  const RigidBody<T>* previous_link = &model_.get_world_body();

  std::stringstream stream;

  // Add rigid bodies for each link.
  for (int link_index = 0; link_index < get_num_links(); ++link_index) {
    const RigidBody<T>& link = model_.template AddBody<RigidBody>(M_Bcm);
    // Make a joint name.
    stream.clear();
    stream << "Joint_" << link_index;
    // Pose of F frame attached on previous link P, measured in P.
    Isometry3d X_PF(Translation3d(0.0, -link_length / 2.0, 0.0));
    // Pose of M frame attached on the new link B, measured in B.
    Isometry3d X_BM(Translation3d(0.0, link_length / 2.0, 0.0));

    const RevoluteJoint<T>& joint =
        model_.template AddJoint<RevoluteJoint>(stream.str(),
        *previous_link, X_PF, link, X_BM, Vector3d::UnitZ());
    joints_.push_back(&joint);

    previous_link = &link;
  }
  DRAKE_ASSERT(model_.get_num_bodies() == get_num_links());
  DRAKE_ASSERT(model_.get_num_joints() == get_num_links());

  model_.template AddForceElement<UniformGravityFieldElement>(
      Vector3d(0.0, -9.81, 0.0));

  model_.Finalize();
}

template<typename T>
void NLinkPendulumPlant<T>::RegisterGeometry(
    geometry::GeometrySystem<T>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);

  std::stringstream stream;

  body_index_to_frame_id_map_.resize(get_num_links());
  source_id_ = geometry_system->RegisterSource("n_link_pendulum");
  // For each body in the model, register a frame with GeometrySystem.
  // Skip the world.
  for (BodyIndex body_index(1);
       body_index < model_.get_num_bodies(); ++body_index) {
    // Define the geometry for each link
    stream.clear();
    stream << "Frame_" << body_index;

    FrameId link_frame_id = geometry_system->RegisterFrame(
        source_id_, GeometryFrame(
            stream.str(), Isometry3<double>::Identity()));
    body_index_to_frame_id_map_[body_index] = link_frame_id;

    // The geometry is right at the frame's origin.
    geometry_system->RegisterGeometry(
        source_id_, link_frame_id,
        std::make_unique<GeometryInstance>(
            Isometry3<double>::Identity(), /* Geometry pose in link's frame */
            std::make_unique<Sphere>(get_radius())));
  }
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
NLinkPendulumPlant<T>::DoMakeContext() const {
  return model_.CreateDefaultContext();
}

template<typename T>
void NLinkPendulumPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = model_.get_num_velocities();

  PositionKinematicsCache<T> pc(model_.get_topology());
  VelocityKinematicsCache<T> vc(model_.get_topology());

  MatrixX<T> M(nv, nv);
  model_.CalcMassMatrixViaInverseDynamics(context, &M);

  // Check if M is symmetric.
  const T err_sym = (M - M.transpose()).norm();
  DRAKE_DEMAND(err_sym < 10 * std::numeric_limits<double>::epsilon());

  model_.CalcPositionKinematicsCache(context, &pc);
  model_.CalcVelocityKinematicsCache(context, pc, &vc);

  // Compute applied forces.
  std::vector<SpatialForce<T>> Fapplied_Bo_W_array(model_.get_num_bodies());
  VectorX<T> tau_applied(model_.get_num_velocities());
  model_.CalcForceElementsContribution(
      context, pc, vc, &Fapplied_Bo_W_array, &tau_applied);
  std::vector<SpatialAcceleration<T>> A_WB_array(model_.get_num_bodies());

  VectorX<T> vdot = VectorX<T>::Zero(nv);
  VectorX<T> C(nv);
  model_.CalcInverseDynamics(
      context, pc, vc, vdot, Fapplied_Bo_W_array, tau_applied,
      &A_WB_array, &Fapplied_Bo_W_array, &C);

  auto v = x.bottomRows(nv);

  VectorX<T> xdot(model_.get_num_states());
  xdot << v, M.llt().solve(-C);
  derivatives->SetFromVector(xdot);
}

template<typename T>
void NLinkPendulumPlant<T>::SetStraightAtAnAngle(
    Context<T>* context, const T& angle) const {
  joints_[0]->set_angle(context, angle);
}

template<typename T>
T NLinkPendulumPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const auto& mbt_context =
      dynamic_cast<const multibody::MultibodyTreeContext<T>&>(context);
  Eigen::VectorBlock<const VectorX<T>> v = mbt_context.get_velocities();

  const int nv = model_.get_num_velocities();

  // TODO(amcastro-tri): Provide an MBT implementation that does not require a
  // mass matrix computation.
  MatrixX<T> M(nv, nv);
  model_.CalcMassMatrixViaInverseDynamics(context, &M);

  return 0.5 * v.transpose() * M * v;
}

template<typename T>
T NLinkPendulumPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  return model_.CalcPotentialEnergy(context);
}

template class NLinkPendulumPlant<double>;
template class NLinkPendulumPlant<AutoDiffXd>;

}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake
