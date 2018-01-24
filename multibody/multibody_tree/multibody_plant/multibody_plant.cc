#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

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
namespace multibody {
namespace multibody_plant {

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
using drake::multibody::RotationalInertia;
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

//template<typename T>
//MultibodyPlant<T>::MultibodyPlant() {
template<typename T>
MultibodyPlant<T>::MultibodyPlant() :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::multibody::multibody_plant::MultibodyPlant>()) {
  model_ = std::make_unique<MultibodyTree<T>>();
}

#if 0
template<typename T>
MultibodyPlant<T>::MultibodyPlant(
    geometry::SourceId source_id,
    std::unordered_map<std::string, geometry::FrameId>& frame_ids) :
    MultibodyPlant() {
  DRAKE_DEMAND(source_id.is_valid());
  source_id_ = source_id;
  //body_name_to_frame_id_ = frame_ids;

  // TODO(amcastro-tri): verify all frame ids actually are part of the
  // geometry system.
}
#endif

template<typename T>
template<typename U>
MultibodyPlant<T>::MultibodyPlant(const MultibodyPlant<U>& other) {
  DRAKE_THROW_UNLESS(!is_finalized());
  model_ = other.model_->template CloneToScalar<T>();
  source_id_ = other.source_id_;
  body_index_to_frame_id_ = other.body_index_to_frame_id_;
  body_name_to_frame_id_ = other.body_name_to_frame_id_;
}

template<typename T>
void MultibodyPlant<T>::RegisterGeometry(
    const Body<T>& body,
    const Isometry3<double>& X_BG, const geometry::Shape& shape,
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_THROW_UNLESS(!this->is_finalized());

  // If not already done, register with the provided geometry system.
  if (!geometry_source_is_registered())
    source_id_ = geometry_system->RegisterSource("MultibodyPlant");

  // If not already done, register a frame for this body.
  if (!body_has_registered_frame(body)) {
    body_index_to_frame_id_[body.get_index()] =
        geometry_system->RegisterFrame(
            source_id_.value(),
            GeometryFrame(
                // TODO: add body names!!
                "Body" + std::to_string(body.get_index()),
                /* Initial pose ??? */
                Isometry3<double>::Identity()));
  }

  // Register geometry in the body frame.
  geometry_system->RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.get_index()],
      std::make_unique<GeometryInstance>(X_BG, shape.Clone()));
}

template<typename T>
void MultibodyPlant<T>::DeclareStateAndPorts() {
  // The model must be finalized.
  DRAKE_DEMAND(this->is_finalized());

  this->DeclareContinuousState(
      BasicVector<T>(model_->get_num_states()),
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);

#if 0
  // Declare a vector input of size one for an applied torque at the
  // elbow joint.
  applied_torque_input_ =
      this->DeclareVectorInputPort(BasicVector<T>(1)).get_index();
#endif

  // Declare a port that outputs the state.
  state_output_port_ = this->DeclareVectorOutputPort(
      BasicVector<T>(model_->get_num_states()),
      &MultibodyPlant::CopyStateOut).get_index();
}

template<typename T>
void MultibodyPlant<T>::DeclareGeometrySystemPorts() {
  geometry_id_port_ =
      this->DeclareAbstractOutputPort(
          &MultibodyPlant::AllocateFrameIdOutput,
          &MultibodyPlant::CalcFrameIdOutput).get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          &MultibodyPlant::AllocateFramePoseOutput,
          &MultibodyPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
FrameIdVector MultibodyPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  // User must be done adding elements to the model.
  DRAKE_DEMAND(model_->topology_is_valid());
  FrameIdVector ids(source_id_.value());
  // Add a frame for the one single body in this model.
  // ids are ordered by body index. This must be consistent with the order in
  // which CalcFramePoseOutput() places the poses in its output.
  // Not all bodies need go have geometry.
  for (auto it : body_index_to_frame_id_)
    ids.AddFrameId(it.second);
  return ids;
}

template <typename T>
void MultibodyPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector*) const {
  // Just a sanity check.
  DRAKE_DEMAND(source_id_ != nullopt);
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
FramePoseVector<T> MultibodyPlant<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  FramePoseVector<T> poses(source_id_.value());
  // Only the pose for bodies for which geometry has been regiestered needs to
  // be placed in the output.
  const int num_bodies_with_geometry = body_index_to_frame_id_.size();
  poses.mutable_vector().resize(num_bodies_with_geometry);
  return poses;
}

template <typename T>
void MultibodyPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_ASSERT(static_cast<int>(poses->vector().size()) == (num_bodies() - 1));

  PositionKinematicsCache<T> pc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);

  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  int pose_index = 0;
  for (const auto it : body_index_to_frame_id_) {
    const BodyIndex body_index = it.first;
    const Body<T>& body = model_->get_body(body_index);
    pose_data[pose_index++] = pc.get_X_WB(body.get_node_index());
  }
}

template <typename T>
void MultibodyPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                     systems::BasicVector<T>* output) const {
  // TODO(amcastro-tri): consider to output a named vector with entries named
  // based on a Joint::GetDofName(joint_dof) method.
  // For now we will just output the continuous state vector.
  const VectorX<T> state_vector = context.get_continuous_state().CopyToVector();
  output->SetFromVector(state_vector);
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_ids_output_port()
const {
  DRAKE_THROW_UNLESS(is_finalized());
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_poses_output_port()
const {
  DRAKE_THROW_UNLESS(is_finalized());
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

#if 0
template <typename T>
const InputPortDescriptor<T>& MultibodyPlant<T>::get_input_port() const {
  return systems::System<T>::get_input_port(applied_torque_input_);
}
#endif

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_state_output_port() const {
  DRAKE_THROW_UNLESS(is_finalized());
  return systems::System<T>::get_output_port(state_output_port_);
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  model_->Finalize();
  DeclareStateAndPorts();
  // Only declare ports to communicate with a GeometrySystem if the plant is
  // provided with a valid source id.
  if (source_id_) DeclareGeometrySystemPorts();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyPlant<T>::DoMakeContext() const {
  DRAKE_THROW_UNLESS(is_finalized());
  return std::make_unique<MultibodyTreeContext<T>>(model_->get_topology());
}

template<typename T>
void MultibodyPlant<T>::DoCalcTimeDerivatives(
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

  // TODO(amcastro-tri): Add in input forces when #7797 lands.
  // Like so: elbow_->AddInTorque(context, get_tau(context), &forces);

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

template class MultibodyPlant<double>;

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

//DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
  //  class drake::multibody::multibody_plant::MultibodyPlant)
