#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

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
  DRAKE_THROW_UNLESS(other.is_finalized());
  model_ = other.model_->template CloneToScalar<T>();
  body_name_to_index_ = other.body_name_to_index_;
  joint_name_to_index_ = other.joint_name_to_index_;
  DeclareStateAndPorts();
  // TODO(amcastro-tri): Declare GeometrySystem ports.
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  model_->Finalize();
  DeclareStateAndPorts();
  // TODO(amcastro-tri): Declare GeometrySystem ports.
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

  // TODO(amcastro-tri): Eval() these from the context.
  PositionKinematicsCache<T> pc(model_->get_topology());
  VelocityKinematicsCache<T> vc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);
  model_->CalcVelocityKinematicsCache(context, pc, &vc);

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
  model_->CalcAllBodyPosesInWorld(context, pc, X_WB);
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
  model_->CalcAllBodySpatialVelocitiesInWorld(
      context, pc, vc, V_WB);
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
    EigenPtr<MatrixX<T>> p_WQi_set, EigenPtr<MatrixX<T>> Jg_WQi) const {
  DRAKE_THROW_UNLESS(p_BQi_set.rows() == 3);
  const int num_points = p_BQi_set.cols();
  DRAKE_THROW_UNLESS(p_WQi_set != nullptr);
  DRAKE_THROW_UNLESS(p_WQi_set->rows() == 3);
  DRAKE_THROW_UNLESS(p_WQi_set->cols() == num_points);
  DRAKE_THROW_UNLESS(Jg_WQi != nullptr);
  DRAKE_THROW_UNLESS(Jg_WQi->rows() == 3 * num_points);
  DRAKE_THROW_UNLESS(Jg_WQi->cols() == num_velocities());
  model_->CalcPointsGeometricJacobianExpressedInWorld(
      context, frame_B, p_BQi_set, p_WQi_set, Jg_WQi);
}

template <typename T>
void MultibodyPlant<T>::CalcPointsAnalyticalJacobianExpressedInWorld(
    const systems::Context<T>& context,
    const Frame<T>& frame_B, const Eigen::Ref<const MatrixX<T>>& p_BQi_set,
    EigenPtr<MatrixX<T>> p_WQi_set, EigenPtr<MatrixX<T>> Ja_WQi) const {
  DRAKE_THROW_UNLESS(p_BQi_set.rows() == 3);
  const int num_points = p_BQi_set.cols();
  DRAKE_THROW_UNLESS(p_WQi_set != nullptr);
  DRAKE_THROW_UNLESS(Ja_WQi != nullptr);

  // Ensure p_WQi_set has the right size and resize, only if needed.
  if (p_WQi_set->rows() != 3 || p_WQi_set->cols() != num_points) {
    p_WQi_set->resize(3, num_points);
  }

  // Ensure Ja_WQi has the right size and resize, only if needed.
  if (Ja_WQi->rows() != 3 * num_points || Ja_WQi->cols() != num_positions()) {
    Ja_WQi->resize(3 * num_points, num_positions());
  }

  //model_->CalcPointsAnalyticalJacobianExpressedInWorld(
    //  context, frame_B, p_BQi_set, p_WQi_set, Ja_WQi);
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::multibody_plant::MultibodyPlant)
