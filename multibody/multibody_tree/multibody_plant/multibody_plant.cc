#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"

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
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::State;

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyTree;
using drake::multibody::MultibodyTreeContext;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;

#include <iostream>
//#define PRINT_VARn(a) std::cout << #a"\n" << a << std::endl;
#define PRINT_VARn(a) (void)a;

template<typename T>
MultibodyPlant<T>::MultibodyPlant() :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::multibody::multibody_plant::MultibodyPlant>()) {
  model_ = std::make_unique<MultibodyTree<T>>();
}

template<typename T>
template<typename U>
MultibodyPlant<T>::MultibodyPlant(const MultibodyPlant<U>& other) {
  DRAKE_THROW_UNLESS(other.is_finalized());
  model_ = other.model_->template CloneToScalar<T>();
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
                body.get_name(),
                /* Initial pose ??? */
                Isometry3<double>::Identity()));
  }

  // Register geometry in the body frame.
  geometry_system->RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.get_index()],
      std::make_unique<GeometryInstance>(X_BG, shape.Clone()));
}

template<typename T>
void MultibodyPlant<T>::RegisterAnchoredGeometry(
    const Isometry3<double>& X_WG, const geometry::Shape& shape,
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_THROW_UNLESS(!this->is_finalized());

  // If not already done, register with the provided geometry system.
  if (!geometry_source_is_registered())
    source_id_ = geometry_system->RegisterSource("MultibodyPlant");

  geometry_system->RegisterAnchoredGeometry(
      source_id_.value(),
      std::make_unique<GeometryInstance>(X_WG, shape.Clone()));
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  model_->AddQuaternionFreeMobilizerToAllBodiesWithNoMobilizer();
  model_->Finalize();
  DeclareStateAndPorts();
  // Only declare ports to communicate with a GeometrySystem if the plant is
  // provided with a valid source id.
  if (source_id_) DeclareGeometrySystemPorts();
  DeclareCacheEntries();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyPlant<T>::DoMakeLeafContext() const {
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
  const int nv = this->num_velocities();

  // Allocate workspace. We might want to cache these to avoid allocations.
  // Mass matrix.
  MatrixX<T> M(nv, nv);
  // Forces.
  MultibodyForces<T> forces(*model_);
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->get_num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Compute forces applied through force elements. This effectively resets
  // the forces to zero and adds in contributions due to force elements.
  model_->CalcForceElementsContribution(context, pc, vc, &forces);

  // TODO(amcastro-tri): Add in input forces with actuators.
  // Like so: elbow_->AddInTorque(context, get_tau(context), &forces);

  model_->CalcMassMatrixViaInverseDynamics(context, &M);

  PRINT_VARn(M);

  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both the array of applied body forces and
  // the array of applied generalized forces get overwritten on output. This is
  // not important in this case since we don't need their values anymore.
  // Please see the documentation for CalcInverseDynamics() for details.

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

  PRINT_VARn(tau_array.transpose());

  vdot = M.ldlt().solve(-tau_array);

  PRINT_VARn(vdot.transpose());

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(this->num_multibody_states());
  VectorX<T> qdot(this->num_positions());
  model_->MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
void MultibodyPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = model_->get_num_positions();
  const int nv = model_->get_num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  VectorX<T> v(nv);
  model_->MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template<typename T>
void MultibodyPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = model_->get_num_positions();
  const int nv = model_->get_num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  model_->MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void MultibodyPlant<T>::DeclareStateAndPorts() {
  // The model must be finalized.
  DRAKE_DEMAND(this->is_finalized());

  this->DeclareContinuousState(
      BasicVector<T>(model_->get_num_states()),
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);

  // TODO(amcastro-tri): Declare input ports for actuators.

  // TODO(amcastro-tri): Declare output port for the state.
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

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  int pose_index = 0;
  for (const auto it : body_index_to_frame_id_) {
    const BodyIndex body_index = it.first;
    const Body<T>& body = model_->get_body(body_index);
    pose_data[pose_index++] = pc.get_X_WB(body.get_node_index());
    PRINT_VARn(pc.get_X_WB(body.get_node_index()).matrix());
  }
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

template<typename T>
void MultibodyPlant<T>::DeclareCacheEntries() {
  // TODO(amcastro-tri): User proper System::Declare() infrastructure to
  // declare cache entries when that lands.
  pc_ = std::make_unique<PositionKinematicsCache<T>>(model_->get_topology());
  vc_ = std::make_unique<VelocityKinematicsCache<T>>(model_->get_topology());
}

template<typename T>
const PositionKinematicsCache<T>& MultibodyPlant<T>::EvalPositionKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace Calc() for an actual Eval() when caching lands.
  model_->CalcPositionKinematicsCache(context, pc_.get());
  return *pc_;
}

template<typename T>
const VelocityKinematicsCache<T>& MultibodyPlant<T>::EvalVelocityKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace Calc() for an actual Eval() when caching lands.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  model_->CalcVelocityKinematicsCache(context, pc, vc_.get());
  return *vc_;
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::multibody_plant::MultibodyPlant)
