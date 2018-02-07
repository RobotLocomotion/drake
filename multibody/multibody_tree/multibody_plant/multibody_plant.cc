#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyTree;
using drake::multibody::MultibodyTreeContext;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;

template<typename T>
MultibodyPlant<T>::MultibodyPlant() :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::multibody::multibody_plant::MultibodyPlant>()) {
  // TODO(amcastro-tri): when caching and MultibodyCacheEvaluatorInterface land
  // the line below should read:
  // model_ = std::make_unique<MultibodyTree<T>>(this);
  model_ = std::make_unique<MultibodyTree<T>>();
}

template<typename T>
MultibodyPlant<T>::MultibodyPlant(std::unique_ptr<MultibodyTree<T>> model) :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::multibody::multibody_plant::MultibodyPlant>()) {
  DRAKE_THROW_UNLESS(model != nullptr);
  model_ = std::move(model);
  // TODO(amcastro-tri): Introduce MultibodyCacheEvaluatorInterface and add a
  // call model_->set_evaluator(this); so that we can use system's caching with
  // no cyclic dependencies.
}

template<typename T>
template<typename U>
MultibodyPlant<T>::MultibodyPlant(const MultibodyPlant<U>& other) {
  DRAKE_THROW_UNLESS(other.is_finalized());
  model_ = other.model_->template CloneToScalar<T>();
  Finalize();
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  if (is_finalized()) return;  // no-op.
  // The MultibodyTree model could have been finalized if coming through
  // as a constructor argument.
  if (!model_->topology_is_valid()) model_->Finalize();
  DeclareStateAndPorts();
  // TODO(amcastro-tri): Declare GeometrySystem ports.
  DeclareCacheEntries();
  plant_is_finalized_ = true;
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

  vdot = M.ldlt().solve(-tau_array);

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(this->num_multibody_states());
  VectorX<T> qdot(this->num_positions());
  model_->MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
void MultibodyPlant<T>::DeclareStateAndPorts() {
  // The model must be finalized.
  DRAKE_DEMAND(model().topology_is_valid());

  this->DeclareContinuousState(
      BasicVector<T>(model_->get_num_states()),
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);

  // TODO(amcastro-tri): Declare input ports for actuators.

  // TODO(amcastro-tri): Declare output port for the state.
}

template <typename T>
void MultibodyPlant<T>::CalcAllBodyPosesInWorld(
    const systems::Context<T>& context,
    std::vector<Isometry3<T>>* X_WB) const {
  DRAKE_THROW_UNLESS(X_WB != nullptr);
  if (static_cast<int>(X_WB->size()) != num_bodies()) {
    X_WB->resize(num_bodies(), Isometry3<T>::Identity());
  }
  // TODO(amcastro-tri): Eval this from the context.
  PositionKinematicsCache<T> pc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const BodyNodeIndex node_index =
        model_->get_body(body_index).get_node_index();
    X_WB->at(body_index) = pc.get_X_WB(node_index);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcAllBodySpatialVelocitiesInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialVelocity<T>>* V_WB) const {
  DRAKE_DEMAND(V_WB != nullptr);
  if (static_cast<int>(V_WB->size()) != num_bodies()) {
    V_WB->resize(num_bodies(), SpatialVelocity<T>::Zero());
  }
  // TODO(amcastro-tri): Eval these from the context.
  PositionKinematicsCache<T> pc(model_->get_topology());
  VelocityKinematicsCache<T> vc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);
  model_->CalcVelocityKinematicsCache(context, pc, &vc);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const BodyNodeIndex node_index =
        model_->get_body(body_index).get_node_index();
    V_WB->at(body_index) = vc.get_V_WB(node_index);
  }
}

template <typename T>
void MultibodyPlant<T>::CalcPointsPositions(
    const systems::Context<T>& context,
    const Frame<T>& from_frame_B,
    const Eigen::Ref<const MatrixX<T>>& p_BQi,
    const Frame<T>& to_frame_A,
    EigenPtr<MatrixX<T>> p_AQi) const {
  DRAKE_THROW_UNLESS(p_BQi.rows() == 3);
  DRAKE_THROW_UNLESS(p_AQi != nullptr);
  DRAKE_THROW_UNLESS(p_AQi->rows() == 3 && p_AQi->cols() == p_BQi.cols());
  model_->CalcPointsPositions(context, from_frame_B, p_BQi, to_frame_A, p_AQi);
}

template <typename T>
void MultibodyPlant<T>::CalcPointsGeometricJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const Frame<T>& frame_B, const Eigen::Ref<const MatrixX<T>>& p_BQi_set,
    EigenPtr<MatrixX<T>> p_WQi_set, EigenPtr<MatrixX<T>> Jv_WQi) const {
  DRAKE_THROW_UNLESS(p_BQi_set.rows() == 3);
  const int num_points = p_BQi_set.cols();
  DRAKE_THROW_UNLESS(p_WQi_set != nullptr);
  DRAKE_THROW_UNLESS(p_WQi_set->rows() == 3);
  DRAKE_THROW_UNLESS(p_WQi_set->cols() == num_points);
  DRAKE_THROW_UNLESS(Jv_WQi != nullptr);
  DRAKE_THROW_UNLESS(Jv_WQi->rows() == 3 * num_points);
  DRAKE_THROW_UNLESS(Jv_WQi->cols() == num_velocities());
  model_->CalcPointsGeometricJacobianExpressedInWorld(
      context, frame_B, p_BQi_set, p_WQi_set, Jv_WQi);
}

template<typename T>
void MultibodyPlant<T>::DeclareCacheEntries() {
  DRAKE_DEMAND(model().topology_is_valid());
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
