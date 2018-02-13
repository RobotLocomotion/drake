#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

//#include <Eigen/SparseCholesky>
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
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::PenetrationAsPointPair;
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
//#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
//#define PRINT_VARn(a) std::cout << #a"\n" << a << std::endl;
#define PRINT_VARn(a) (void)a;
#define PRINT_VAR(a) (void)a;

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
void MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body,
    const Isometry3<double>& X_BG, const geometry::Shape& shape,
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_THROW_UNLESS(!this->is_finalized());
  GeometryId id = RegisterGeometry(body, X_BG, shape, geometry_system);
  geometry_id_to_visual_index_[id] =
      static_cast<int>(geometry_id_to_visual_index_.size());
}

template<typename T>
void MultibodyPlant<T>::RegisterAnchoredVisualGeometry(
    const Isometry3<double>& X_WG, const geometry::Shape& shape,
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_THROW_UNLESS(!this->is_finalized());
  GeometryId id = RegisterAnchoredGeometry(X_WG, shape, geometry_system);
  geometry_id_to_visual_index_[id] =
      static_cast<int>(geometry_id_to_visual_index_.size());
}

template<typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterGeometry(
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
  GeometryId geometry_id = geometry_system->RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.get_index()],
      std::make_unique<GeometryInstance>(X_BG, shape.Clone()));
  geometry_id_to_body_index_[geometry_id] = body.get_index();
  return geometry_id;
}

template<typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterAnchoredGeometry(
    const Isometry3<double>& X_WG, const geometry::Shape& shape,
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_THROW_UNLESS(!this->is_finalized());

  // If not already done, register with the provided geometry system.
  if (!geometry_source_is_registered())
    source_id_ = geometry_system->RegisterSource("MultibodyPlant");

  GeometryId geometry_id = geometry_system->RegisterAnchoredGeometry(
      source_id_.value(),
      std::make_unique<GeometryInstance>(X_WG, shape.Clone()));
  geometry_id_to_body_index_[geometry_id] = world_index();
  return geometry_id;
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
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

  // Compute contact forces on each body by penalty method.
  CalcAndAddContactForcesByPenaltyMethod(context, pc, vc, &F_BBo_W_array);

  model_->CalcInverseDynamics(
      context, pc, vc, vdot,
      F_BBo_W_array, tau_array,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau_array);

  PRINT_VARn(tau_array.transpose());

  vdot = M.ldlt().solve(-tau_array);

  // Compile time error: "The SparseCholesky module has nothing to offer in MPL2 only mode"
  // You need to include: #include <Eigen/SparseCholesky>
#if 0
  const auto& Ms = M.sparseView();
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<T>, Eigen::Lower> solver;
  solver.compute(Ms);
  DRAKE_DEMAND(solver.info() == Eigen::Success);
  vdot = solver.solve(-tau_array);
#endif

  PRINT_VARn(vdot.transpose());

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(this->num_multibody_states());
  VectorX<T> qdot(this->num_positions());
  model_->MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<>
void MultibodyPlant<double>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<double>& context,
    const PositionKinematicsCache<double>& pc, const VelocityKinematicsCache<double>& vc,
    std::vector<SpatialForce<double>>* F_BBo_W_array) const {
  if (get_num_collision_geometries() == 0) return;

  const geometry::QueryObject<double>& query_object =
      this->EvalAbstractInput(context, geometry_query_port_)
          ->template GetValue<geometry::QueryObject<double>>();
  (void) query_object;

  std::vector<PenetrationAsPointPair<double>> penetrations =
      query_object.ComputePointPairPenetration();
  PRINT_VAR(penetrations.size());
  if (penetrations.size() > 0) {
    for (const auto& penetration : penetrations) {
      const GeometryId geometryA_id = penetration.id_A;
      const GeometryId geometryB_id = penetration.id_B;

      // TODO(amcastro-tri): Request GeometrySystem to do this filtering for us
      // when that capability lands.
      if (!is_collision_geometry(geometryA_id) ||
          !is_collision_geometry(geometryB_id))
        continue;

      // NOTE: for now assume this MBP is the only system connected to GS.
      BodyIndex bodyA_index = geometry_id_to_body_index_.at(penetration.id_A);
      BodyIndex bodyB_index = geometry_id_to_body_index_.at(penetration.id_B);

      const Body<double>& bodyA = model().get_body(bodyA_index);
      const Body<double>& bodyB = model().get_body(bodyB_index);

      BodyNodeIndex bodyA_node_index =
          model().get_body(bodyA_index).get_node_index();
      BodyNodeIndex bodyB_node_index =
          model().get_body(bodyB_index).get_node_index();

      // Penetration depth, > 0 during penetration.
      const double& x = penetration.depth;
      const Vector3<double>& nhat_BA_W = penetration.nhat_BA_W;
      const Vector3<double>& p_WCa = penetration.p_WCa;
      const Vector3<double>& p_WCb = penetration.p_WCb;

      // Contact point C.
      const Vector3<double> p_WC = 0.5 * (p_WCa + p_WCb);

      // Contact point position on body A.
      const Vector3<double>& p_WAo = pc.get_X_WB(bodyA_node_index).translation();
      const Vector3<double>& p_CoAo_W = p_WAo - p_WC;

      // Contact point position on body B.
      const Vector3<double>& p_WBo = pc.get_X_WB(bodyB_node_index).translation();
      const Vector3<double>& p_CoBo_W = p_WBo - p_WC;

      // Separation velocity, > 0  if objects separate.
      const Vector3<double> v_WAc =
          vc.get_V_WB(bodyA_node_index).Shift(-p_CoAo_W).translational();
      const Vector3<double> v_WBc =
          vc.get_V_WB(bodyB_node_index).Shift(-p_CoBo_W).translational();
      const Vector3<double> v_AcBc_W = v_WBc - v_WAc;

      // xdot = vn > 0 ==> they are getting closer.
      const double vn = v_AcBc_W.dot(nhat_BA_W);


      // Penetration rate, > 0 implies increasing penetration.
      //const T& xdot = -state.zdot();
      //fC = k_ * x * (1.0 + d_ * xdot);

      // Magnitude of the normal force on body A at contact point C.
      const double& k = contact_penalty_stiffness_;
      const double& d = contact_penalty_damping_;
      const double fn_AC = k * x * (1.0 + d * vn); // xdot = -vn

      PRINT_VAR(bodyA.get_name());
      PRINT_VAR(bodyB.get_name());
      PRINT_VAR(x);
      PRINT_VAR(nhat_BA_W.transpose());
      PRINT_VAR(v_AcBc_W.transpose());
      PRINT_VAR(vn);
      PRINT_VAR(p_WCa.transpose());
      PRINT_VAR(p_WCb.transpose());
      PRINT_VAR(p_WC.transpose());

      if (fn_AC <= 0) continue;  // Continue with next point.

      // Spatial force on body A at C, expressed in the world frame W.
      const SpatialForce<double> F_AC_W(Vector3<double>::Zero(), fn_AC * nhat_BA_W);

      if (bodyA_index != world_index()) {
        // Spatial force on body A at Ao, expressed in W.
        const SpatialForce<double> F_AAo_W = F_AC_W.Shift(p_CoAo_W);
        F_BBo_W_array->at(bodyA_index) += F_AAo_W;
      }

      if (bodyB_index != world_index()) {
        // Spatial force on body B at Bo, expressed in W.
        const SpatialForce<double> F_BBo_W = -F_AC_W.Shift(p_CoBo_W);
        F_BBo_W_array->at(bodyB_index) += F_BBo_W;
      }
    }
  }  // if (penetrations.size() > 0)
}

template<typename T>
void MultibodyPlant<T>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc, const VelocityKinematicsCache<T>& vc,
    std::vector<SpatialForce<T>>* F_BBo_W_array) const {
  DRAKE_ABORT_MSG("Only <double> is supported.");
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
  geometry_query_port_ = this->DeclareAbstractInputPort().get_index();
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
    //PRINT_VARn(pc.get_X_WB(body.get_node_index()).matrix());
    PRINT_VAR(body_index);
    PRINT_VAR(pc.get_X_WB(body.get_node_index()).matrix());
  }
}

template <typename T>
const systems::InputPortDescriptor<T>&
MultibodyPlant<T>::get_geometry_query_input_port() const {
  return systems::System<T>::get_input_port(geometry_query_port_);
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
