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
  model_ = std::make_unique<MultibodyTree<T>>();
}

template<typename T>
template<typename U>
MultibodyPlant<T>::MultibodyPlant(const MultibodyPlant<U>& other) {
  DRAKE_THROW_UNLESS(!is_finalized());
  model_ = other.model_->template CloneToScalar<T>();
  body_name_to_index_ = other.body_name_to_index_;
  joint_name_to_index_ = other.joint_name_to_index_;
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  model_->Finalize();
  DeclareStateAndPorts();
  // TODO(amcastro-tri): Declare GeometrySystem ports.
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
  DRAKE_DEMAND(this->is_finalized());

  this->DeclareContinuousState(
      BasicVector<T>(model_->get_num_states()),
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);

  // TODO(amcastro-tri): Declare input ports for actuators.

  // TODO(amcastro-tri): Declare output port for the state.
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
