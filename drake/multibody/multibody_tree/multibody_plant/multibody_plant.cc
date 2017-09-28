#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace multibody {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::FrameId;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::Sphere;
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
MultibodyPlant<T>::MultibodyPlant(
    std::unique_ptr<MultibodyTree<T>> model,
    const std::string& name,
    geometry::GeometrySystem<T>* geometry_system) {
  model_ = std::move(model);
  DRAKE_DEMAND(get_num_positions() != 0);
  DRAKE_DEMAND(get_num_velocities() != 0);
  this->set_name(name);

  DeclareState();
  DeclareOutputPortsForGeometrySystem(geometry_system);
  // TODO: add output ports for kinetic, potential and total energy.
  // See CosseratRodPlant for an example passing down lambdas.
}

template<typename T>
MultibodyPlant<T>::MultibodyPlant(const std::string& name) {
  this->set_name(name);
  model_ = std::make_unique<MultibodyTree<T>>();
}

template<typename T>
void MultibodyPlant<T>::Init(geometry::GeometrySystem<T>* geometry_system) {
  BuildMultibodyModel(model_.get());
  RegisterGeometrySystemFrames(geometry_system);
  RegisterGeometry(geometry_system);
  // Ensures sub-classes didn't forget to Finalize() the model.
  if (!model_->topology_is_valid()) model_->Finalize();
  // Demand a non-empty model.
  DRAKE_DEMAND(get_num_positions() != 0);
  DRAKE_DEMAND(get_num_velocities() != 0);
  DeclareState();
  DeclareOutputPortsForGeometrySystem(geometry_system);
}

template<typename T>
void MultibodyPlant<T>::RegisterGeometrySystemFrames(
    geometry::GeometrySystem<T>* geometry_system) {
  if (geometry_system != nullptr) {
    source_id_ = geometry_system->RegisterSource(this->get_name());

    // TODO(amcastro-tri): get names from bodies. Make sure they are unique!
    // Since these name frames that must be unique at least for visualization.
    std::stringstream stream;
    // Id's are all invalid after resize().
    body_index_to_frame_id_map_.resize(get_num_bodies());
    body_index_to_frame_index_map_.resize(get_num_bodies());
    body_index_to_frame_index_map_[world_index()] = -1;  // invalid index.
    // Skip the world, therfore it's frame id will remain invalid.
    int frame_index = 0;
    for (BodyIndex body_index(1); body_index < get_num_bodies(); ++body_index) {
      stream.clear();
      stream << "Body_" << body_index;
      FrameId link_frame_id = geometry_system->RegisterFrame(
          source_id_, GeometryFrame(
              stream.str(), Isometry3<double>::Identity()));
      body_index_to_frame_id_map_[body_index] = link_frame_id;
      body_index_to_frame_index_map_[body_index] = frame_index++;
    }
  }
}

template <typename T>
void MultibodyPlant<T>::RegisterGeometry(
    const Body<T>& body, const Isometry3<double>& X_BG,
    std::unique_ptr<geometry::Shape> shape,
    geometry::GeometrySystem<T>* geometry_system) const {
  body.HasThisParentTreeOrThrow(model_.get());
  // The geometry is right at the frame's origin.
  FrameId frame_id = body_index_to_frame_id_map_[body.get_index()];
  geometry_system->RegisterGeometry(
      source_id_, frame_id,
      std::make_unique<GeometryInstance>(X_BG, std::move(shape)));
}

template<typename T>
int MultibodyPlant<T>::get_num_bodies() const {
  return model_->get_num_bodies();
}

template<typename T>
int MultibodyPlant<T>::get_num_positions() const {
  return model_->get_num_positions();
}

template<typename T>
int MultibodyPlant<T>::get_num_velocities() const {
  return model_->get_num_velocities();
}

template<typename T>
int MultibodyPlant<T>::get_num_states() const {
  return model_->get_num_states();
}

template<typename T>
void MultibodyPlant<T>::DeclareState() {
  DRAKE_ASSERT(model_ != nullptr);

  // For subclasses, demand users to have an already finalized model.
  DRAKE_DEMAND(model_->topology_is_valid());

  // TODO(amcastro-tri): consider adding z-state for tracking energy
  // conservation.
  this->DeclareContinuousState(
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);
}

template<typename T>
void MultibodyPlant<T>::DeclareOutputPortsForGeometrySystem(
    geometry::GeometrySystem<T>* geometry_system) {
  if (geometry_system != nullptr) {
    geometry_id_port_ =
        this->DeclareAbstractOutputPort(
            &MultibodyPlant::AllocateFrameIdOutput,
            &MultibodyPlant::CalcFrameIdOutput).get_index();
    geometry_pose_port_ =
        this->DeclareAbstractOutputPort(
            &MultibodyPlant::AllocateFramePoseOutput,
            &MultibodyPlant::CalcFramePoseOutput).get_index();
  }
}

template <typename T>
FrameIdVector MultibodyPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_.is_valid());
  FrameIdVector ids(source_id_);
  // Frame ids for GeometrySystem do not contain the "world" frame, skip it.
  for (BodyIndex body_index(1); body_index < get_num_bodies(); ++body_index) {
    ids.AddFrameId(body_index_to_frame_id_map_[body_index]);
  }
  return ids;
}

template <typename T>
void MultibodyPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector*) const {
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
FramePoseVector<T> MultibodyPlant<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_.is_valid());
  FramePoseVector<T> poses(source_id_);
  // Poses for GeometrySystem do not contain the world's pose, do not count it.
  poses.mutable_vector().resize(get_num_bodies() - 1);
  return poses;
}

template <typename T>
void MultibodyPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_ASSERT(
      static_cast<int>(poses->vector().size()) == get_num_bodies() - 1);

  // TODO(amcastro-tri): move this to the cache.
  PositionKinematicsCache<T> pc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);

  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  // GeometrySystem poses do not contain the world's pose, skip it.
  for (BodyIndex body_index(1); body_index < get_num_bodies(); ++body_index) {
    const int frame_index = body_index_to_frame_index_map_[body_index];
    const BodyNodeIndex node_index =
        model_->get_body(body_index).get_node_index();
    pose_data[frame_index] = pc.get_X_WB(node_index);
  }
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_id_output_port()
const {
  DRAKE_DEMAND(geometry_id_port_ >= 0);
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_pose_output_port()
const {
  DRAKE_DEMAND(geometry_pose_port_ >= 0);
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template<typename T>
template<typename U>
MultibodyPlant<T>::MultibodyPlant(
    const MultibodyPlant<U>& other) :
    MultibodyPlant<T>(this->get_name()) {
  model_ = other.model_->template CloneToScalar<T>();
  source_id_ = other.source_id_;
  body_index_to_frame_id_map_ = other.body_index_to_frame_id_map_;
  body_index_to_frame_index_map_ = other.body_index_to_frame_index_map_;
  geometry_id_port_ = other.geometry_id_port_;
  geometry_pose_port_ = other.geometry_pose_port_;
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyPlant<T>::DoMakeContext() const {
  return model_->CreateDefaultContext();
}

template<typename T>
void MultibodyPlant<T>::DoCalcTimeDerivatives(
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

  // Check if M is symmetric.
  const T err_sym = (M - M.transpose()).norm();
  DRAKE_ASSERT(err_sym < 10 * std::numeric_limits<double>::epsilon());

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

  // TODO(amcastro-tri): add v to qdot mapping.
  VectorX<T> xdot(model_->get_num_states());
  xdot << v, M.llt().solve(-C);
  derivatives->SetFromVector(xdot);
}

template<typename T>
T MultibodyPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  // TODO: make this an MBT method.
  // Use it also in DoCalcTimeDerivatives()
  const auto& mbt_context =
      dynamic_cast<const multibody::MultibodyTreeContext<T>&>(context);
  Eigen::VectorBlock<const VectorX<T>> v = mbt_context.get_velocities();

  const int nv = model_->get_num_velocities();

  // TODO(amcastro-tri): provide an implementation that does not require to
  // compute the mass matrix.
  MatrixX<T> M(nv, nv);
  model_->CalcMassMatrixViaInverseDynamics(context, &M);
  return 0.5 * v.transpose() * M * v;
}

template<typename T>
T MultibodyPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  return model_->CalcPotentialEnergy(context);
}

template class MultibodyPlant<double>;
//template class MultibodyPlant<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
