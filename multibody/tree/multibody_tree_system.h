#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/articulated_body_force_cache.h"
#include "drake/multibody/tree/articulated_body_inertia_cache.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/cache_entry.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class MultibodyTree;

template <typename T>
class MultibodyTreeSystemElementAttorney;

/* This is a bare Drake System providing just enough functionality to allow
standalone exercise of a MultibodyTree. A Drake System must implement
several virtual methods, and MultibodyTree requires System services
to allocate and access the resources it needs in a Context. All
resources that MultibodyTree needs are hosted here so that
MultibodyTree elements can get access to them. Derived classes (likely
MultibodyPlant) may have their own context resources but those will not be
accessible from MultibodyTree.

%MultibodyTreeSystem serves as the base class for the MultibodyPlant System,
which provides much more functionality, including full integration with the
Drake System Framework via input and output ports, contact modeling, and a
discrete solver for forward dynamics. %MultibodyTreeSystem alone is useful for
unit testing of MultibodyTree, and on those rare occasions where nothing but
tree functionality is needed.

To use %MultibodyTreeSystem alone, first create and populate a MultibodyTree,
then transfer ownership of it in the constructor for %MultibodyTreeSystem, which
will finalize the tree if that hasn't already been done, interrogate it for the
Context resources it needs, and allocate them. No further changes are possible
to the MultibodyTree once it is owned by %MultibodyTreeSystem. For example,
@code{.cpp}
  // Create an empty model.
  auto mb_tree = std::make_unique<MultibodyTree<double>>
  mb_tree->AddBody<RigidBody>(...);
  mb_tree->AddMobilizer<RevoluteMobilizer>(...);
  // ...
  // Done adding modeling elements. Transfer tree to system, get Context.
  auto system = std::make_unique<MultibodyTreeSystem<double>>(std::move(model));
  auto context = system->CreateDefaultContext();
@endcode

Derived classes (likely MultibodyPlant) may use an alternate protected interface
that provides for incremental construction of the MultibodyTree owned by a
MultibodyTreeSystem. See documentation for the protected methods below, and look
at MultibodyPlant for an example. */
template <typename T>
class MultibodyTreeSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeSystem)

  /* Takes ownership of the given `tree`, finalizes it if it hasn't already
  been finalized, and then allocates the resources it needs. You cannot modify
  the tree after that. The `tree` cannot be null.

  @param[in] tree        An already-complete MultibodyTree.
  @param[in] is_discrete Whether to allocate discrete state variables for the
      MultibodyTree kinematics. Otherwise allocates continuous state variables.

  @throws std::exception if `tree` is null. */
  explicit MultibodyTreeSystem(std::unique_ptr<MultibodyTree<T>> tree,
                               bool is_discrete = false);

  /* Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit MultibodyTreeSystem(const MultibodyTreeSystem<U>& other);

  ~MultibodyTreeSystem() override;

  bool is_discrete() const { return is_discrete_; }

  /* Returns a reference to the up-to-date PositionKinematicsCache in the
  given Context, recalculating it first if necessary. */
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return position_kinematics_cache_entry()
        .template Eval<PositionKinematicsCache<T>>(context);
  }

  /* Returns a reference to the up-to-date VelocityKinematicsCache in the
  given Context, recalculating it first if necessary. Also if necessary, the
  PositionKinematicsCache will be recalculated as well. */
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return velocity_kinematics_cache_entry()
        .template Eval<VelocityKinematicsCache<T>>(context);
  }

  /* Returns a reference to the up-to-date AccelerationKinematicsCache in the
  given Context, recalculating it first via forward dynamics if necessary. Also
  if necessary, other cache entries such as PositionKinematicsCache and
  VelocityKinematicsCache will be recalculated as well. */
  const internal::AccelerationKinematicsCache<T>& EvalForwardDynamics(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return acceleration_kinematics_cache_entry()
        .template Eval<AccelerationKinematicsCache<T>>(context);
  }

  /* Returns a reference to the up-to-date ArticulatedBodyInertiaCache stored
  in the given context, recalculating it first if necessary.
  See @ref internal_forward_dynamics
  "Articulated Body Algorithm Forward Dynamics" for further details. */
  const ArticulatedBodyInertiaCache<T>& EvalArticulatedBodyInertiaCache(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.abi_cache_index)
        .template Eval<ArticulatedBodyInertiaCache<T>>(context);
  }

  /* Returns a reference to the up-to-date cache of per-body spatial inertias
  in the given Context, recalculating it first if necessary. */
  const std::vector<SpatialInertia<T>>& EvalSpatialInertiaInWorldCache(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.spatial_inertia_in_world)
        .template Eval<std::vector<SpatialInertia<T>>>(context);
  }

  /* Returns a reference to the up-to-date cache of per-dof reflected inertias
  in the given Context, recalculating it first if necessary. */
  const VectorX<T>& EvalReflectedInertiaCache(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.reflected_inertia)
        .template Eval<VectorX<T>>(context);
  }

  /* Returns a reference to the up-to-date cache of composite-body inertias
  in the given Context, recalculating it first if necessary. */
  const std::vector<SpatialInertia<T>>& EvalCompositeBodyInertiaInWorldCache(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.composite_body_inertia_in_world)
        .template Eval<std::vector<SpatialInertia<T>>>(context);
  }

  /* Returns a reference to the up-to-date cache of per-body bias terms in
  the given Context, recalculating it first if necessary.
  For a body B, this is the bias term `Fb_Bo_W(q, v)` in the equation
  `F_Bo_W = M_Bo_W * A_WB + Fb_Bo_W`, where `M_Bo_W` is the spatial inertia
  about B's origin Bo, `A_WB` is the spatial acceleration of B in W and
  `F_Bo_W` is the spatial force on B about Bo, expressed in W. */
  const std::vector<SpatialForce<T>>& EvalDynamicBiasCache(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.dynamic_bias)
        .template Eval<std::vector<SpatialForce<T>>>(context);
  }

  /* Returns a reference to the up-to-date cache of per-body spatial
  acceleration bias terms in the given Context, recalculating it first if
  necessary. For a body B, this is the spatial acceleration bias term
  `Ab_WB(q, v)`, function of both q and v, as it appears in the acceleration
  level motion constraint imposed by body B's mobilizer
  `A_WB = Aplus_WB + Ab_WB + H_PB_W * vdot_B`, with
  `Aplus_WB = Φᵀ(p_PB) * A_WP` the rigidly shifted spatial acceleration of the
  inboard body P and `H_PB_W` and `vdot_B` its mobilizer's hinge matrix and
  mobility time derivative, respectively.
  See @ref abi_computing_accelerations for further details. */
  const std::vector<SpatialAcceleration<T>>& EvalSpatialAccelerationBiasCache(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.spatial_acceleration_bias)
        .template Eval<std::vector<SpatialAcceleration<T>>>(context);
  }

  /* For a body B, this evaluates the velocity-dependent articulated body force
  bias `Zb_Bo_W(q, v) = Pplus_PB_W(q) * Ab_WB(q, v)`. This computation is
  particularly expensive when performing O(n) forward dynamics with different
  applied forces but with the same multibody state x = [q, v] and therefore it
  is worth caching. */
  const std::vector<SpatialForce<T>>&
  EvalArticulatedBodyForceBiasCache(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.articulated_body_force_bias)
        .template Eval<std::vector<SpatialForce<T>>>(context);
  }

  /* When using the articulated body algorithm, this cache entry holds the
  per-body and per-dof propagated forces. These include the effects of both
  the velocity-dependent bias forces and applied forces. */
  const ArticulatedBodyForceCache<T>&
  EvalArticulatedBodyForceCache(const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.articulated_body_forces)
        .template Eval<ArticulatedBodyForceCache<T>>(context);
  }

  /* For a body B connected to its parent P, returns a reference to the up to
  date cached value for H_PB_W, where H_PB_W is the `6 x nm` body-node hinge
  matrix that relates `V_PB_W` (body B's spatial velocity in its parent body P,
  expressed in world W) to this node's `nm` generalized velocities
  (or mobilities) `v_B` as `V_PB_W = H_PB_W * v_B`.
  As needed, H_PB_W is recalculated from the context and the
  PositionKinematicsCache may also be recalculated (since it stores H_FM(q) for
  each mobilizer and X_WB(q) for each body). The returned `std::vector` stores
  all the body-node hinge matrices in the tree as a vector of the columns of
  these matrices. Therefore the returned `std::vector` of columns has as many
  entries as number of generalized velocities in the tree. */
  const std::vector<Vector6<T>>&
  EvalAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context) const {
    this->ValidateContext(context);
    return this->get_cache_entry(cache_indexes_.across_node_jacobians)
        .template Eval<std::vector<Vector6<T>>>(context);
  }

  /* Returns the cache entry that holds position kinematics results. */
  const systems::CacheEntry& position_kinematics_cache_entry() const {
    return this->get_cache_entry(cache_indexes_.position_kinematics);
  }

  /* Returns the cache entry that holds velocity kinematics results. */
  const systems::CacheEntry& velocity_kinematics_cache_entry() const {
    return this->get_cache_entry(cache_indexes_.velocity_kinematics);
  }

  /* Returns the cache entry that holds acceleration kinematics results. */
  const systems::CacheEntry& acceleration_kinematics_cache_entry() const {
    return this->get_cache_entry(cache_indexes_.acceleration_kinematics);
  }

  /* Returns the DiscreteStateIndex for the one and only multibody discrete
  state if the system is discrete and finalized. Throws otherwise. */
  systems::DiscreteStateIndex GetDiscreteStateIndexOrThrow() const {
    if (!is_discrete_) {
      throw std::runtime_error(
          "The MultibodyTreeSystem is modeled as a continuous system and there "
          "does not exist any discrete state.");
    }
    if (!already_finalized_) {
      throw std::logic_error(
          "GetDiscreteStateIndexOrThrow() can only be "
          "called post-Finalize().");
    }
    return tree_->get_discrete_state_index();
  }

 protected:
  /* @name        Alternate API for derived classes
  Derived classes may use these methods to create a MultibodyTreeSystem
  that owns an empty MultibodyTree, then incrementally build it, and finalize
  it when done. See MultibodyPlant for a working example. */
  //@{

  /* Default constructor allocates a MultibodyTree, with the intent that it
  will be filled in later, using mutable_tree() for access. You must call
  Finalize() when done before performing any computations. */
  explicit MultibodyTreeSystem(bool is_discrete = false);

  /*  Constructor that specifies scalar-type conversion support.
  If `tree` is given, we'll finalize it. Otherwise, we'll allocate an
  empty one and leave it not finalized.
  @param[in] converter Scalar-type conversion support helper.
  @param[in] tree Already-complete MultibodyTree if supplied. If nullptr, an
      empty MultibodyTree is allocated internally.
  @param[in] is_discrete Whether to use discrete state variables for tree
      kinematics. Otherwise uses continuous state variables q and v. */
  MultibodyTreeSystem(systems::SystemScalarConverter converter,
                      std::unique_ptr<MultibodyTree<T>> tree,
                      bool is_discrete = false);

  template <typename U>
  friend const MultibodyTree<U>& GetInternalTree(
      const MultibodyTreeSystem<U>&);

  /* Returns a const reference to the MultibodyTree owned by this class. */
  const MultibodyTree<T>& internal_tree() const {
    DRAKE_ASSERT(tree_ != nullptr);
    return *tree_;
  }

  /* Returns a mutable reference to the MultibodyTree owned by this class. */
  MultibodyTree<T>& mutable_tree() const;

  /* Finalize the tree if that hasn't already been done, complete System
  construction, and declare any needed Context resources for the tree. You must
  call this before performing any computation. */
  void Finalize();

  /* Derived class (likely MultibodyPlant) must implement this if it has
  forces to apply other than those applied internally by elements of the
  MultibodyTree. `forces` has already been initialized and may contain
  force contributions already; be sure to add rather than overwrite. This
  method will only be called in continuous mode. */
  virtual void AddInForcesContinuous(const systems::Context<T>& context,
                                     MultibodyForces<T>* forces) const {
    unused(context, forces);
  }

  /* Derived class (likely MultibodyPlant) must implement this to support
  forward dynamics when in discrete mode. */
  virtual void DoCalcForwardDynamicsDiscrete(
      const systems::Context<T>& context,
      AccelerationKinematicsCache<T>* ac) const {
    unused(context, ac);
    throw std::logic_error(
        "DoCalcForwardDynamicsDiscrete(): invoked but not implemented.");
  }
  //@}

  // TODO(sherm1) Shouldn't require overriding the default method; need
  // a DoLeafSetDefaultState().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  // This is only meaningful in continuous mode.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const final;

  void DoMapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      systems::VectorBase<T>* generalized_velocity) const final;

  void DoMapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      systems::VectorBase<T>* qdot) const final;

  // Public documentation for this overload can be found in multibody_plant.h.
  void DoCalcImplicitTimeDerivativesResidual(
      const systems::Context<T>& context,
      const systems::ContinuousState<T>& proposed_derivatives,
      EigenPtr<VectorX<T>> residual) const final;

  T DoCalcPotentialEnergy(const systems::Context<T>& context) const final {
    return internal_tree().CalcPotentialEnergy(context);
  }

  T DoCalcKineticEnergy(const systems::Context<T>& context) const final {
    return internal_tree().CalcKineticEnergy(context);
  }

  T DoCalcConservativePower(const systems::Context<T>& context) const final {
    return internal_tree().CalcConservativePower(context);
  }

  T DoCalcNonConservativePower(
      const systems::Context<T>& context) const final {
    return internal_tree().CalcNonConservativePower(context);
  }

  // Configuration-dependent cached computations.

  void CalcPositionKinematicsCache(
      const systems::Context<T>& context,
      PositionKinematicsCache<T>* position_cache) const {
    internal_tree().CalcPositionKinematicsCache(context, position_cache);
  }

  void CalcSpatialInertiasInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialInertia<T>>* spatial_inertias) const {
    internal_tree().CalcSpatialInertiasInWorld(context, spatial_inertias);
  }

  void CalcReflectedInertia(const systems::Context<T>& context,
                            VectorX<T>* reflected_inertia) const {
    internal_tree().CalcReflectedInertia(context, reflected_inertia);
  }

  void CalcCompositeBodyInertiasInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialInertia<T>>* composite_body_inertias) const {
    internal_tree().CalcCompositeBodyInertiasInWorld(context,
                                                     composite_body_inertias);
  }

  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context,
      std::vector<Vector6<T>>* H_PB_W_all) const {
    internal_tree().CalcAcrossNodeJacobianWrtVExpressedInWorld(
        context, EvalPositionKinematics(context), H_PB_W_all);
  }

  void CalcArticulatedBodyInertiaCache(
      const systems::Context<T>& context,
      ArticulatedBodyInertiaCache<T>* abi_cache) const {
    internal_tree().CalcArticulatedBodyInertiaCache(context, abi_cache);
  }

  // Velocity-dependent cached computations. (These also depend on
  // configuration.)

  void CalcVelocityKinematicsCache(
      const systems::Context<T>& context,
      VelocityKinematicsCache<T>* velocity_cache) const {
    internal_tree().CalcVelocityKinematicsCache(context,
        EvalPositionKinematics(context), velocity_cache);
  }

  void CalcDynamicBiasForces(
      const systems::Context<T>& context,
      std::vector<SpatialForce<T>>* dynamic_bias_forces) const {
    internal_tree().CalcDynamicBiasForces(context, dynamic_bias_forces);
  }

  void CalcSpatialAccelerationBias(
      const systems::Context<T>& context,
      std::vector<SpatialAcceleration<T>>* Ab_WB_all) const {
    internal_tree().CalcSpatialAccelerationBias(context, Ab_WB_all);
  }

  void CalcArticulatedBodyForceBias(
      const systems::Context<T>& context,
      std::vector<SpatialForce<T>>* Zb_Bo_W_all) const {
    internal_tree().CalcArticulatedBodyForceBias(context, Zb_Bo_W_all);
  }

  // Force-dependent cached computations. (These also depend on configuration
  // and velocity.)

  // Performs an O(n) tip-to-base recursion to compute forces Z_B and
  // Zplus_B, among other quantities needed by ABA.
  // N.B. Please refer to @ref internal_forward_dynamics for further details on
  // the algorithm and implementation.
  void CalcArticulatedBodyForceCache(
      const systems::Context<T>& context,
      ArticulatedBodyForceCache<T>* aba_force_cache) const;

  // This is the method used to evaluate the AccelerationKinematicsCache when
  // it is out of date. The actual computation performed depends on whether
  // we are in continuous or discrete mode.
  void CalcForwardDynamics(const systems::Context<T>& context,
                           AccelerationKinematicsCache<T>* ac) const {
    if (is_discrete())
      CalcForwardDynamicsDiscrete(context, ac);
    else
      CalcForwardDynamicsContinuous(context, ac);
  }

  // When in continuous mode, this method is used to compute forward dynamics.
  // It collects forces from both MultibodyTree internal elements and
  // continuous-mode forces applied by the derived class (likely
  // MultibodyPlant). Then it uses the O(n) Articulated Body Algorithm (ABA)
  // to compute accelerations. Please refer to @ref internal_forward_dynamics
  // for further details on the algorithm and implementation.
  void CalcForwardDynamicsContinuous(
      const systems::Context<T>& context,
      internal::AccelerationKinematicsCache<T>* ac) const;

  // Discrete mode forward dynamics must be implemented by a derived class
  // (likely MultibodyPlant).
  void CalcForwardDynamicsDiscrete(
      const systems::Context<T>& context,
      AccelerationKinematicsCache<T>* ac) const {
    DRAKE_DEMAND(ac != nullptr);
    DRAKE_DEMAND(is_discrete());
    DoCalcForwardDynamicsDiscrete(context, ac);
  }

  // This method is called during Finalize(). It tells each MultibodyElement
  // owned by `this` system to declare their system parameters on `this`.
  void DeclareMultibodyElementParameters();

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U>
  friend class MultibodyTreeSystem;

  friend class MultibodyTreeSystemElementAttorney<T>;

  // This struct stores in one single place all indexes related to
  // MultibodyTreeSystem specific cache entries.
  struct CacheIndexes {
    systems::CacheIndex abi_cache_index;
    systems::CacheIndex acceleration_kinematics;
    systems::CacheIndex across_node_jacobians;
    systems::CacheIndex articulated_body_forces;
    systems::CacheIndex articulated_body_force_bias;
    systems::CacheIndex dynamic_bias;
    systems::CacheIndex position_kinematics;
    systems::CacheIndex spatial_inertia_in_world;
    systems::CacheIndex composite_body_inertia_in_world;
    systems::CacheIndex spatial_acceleration_bias;
    systems::CacheIndex velocity_kinematics;
    systems::CacheIndex reflected_inertia;
  };

  // This is the one real constructor. From the public API, a null tree is
  // illegal and gets an error message. From the protected API, a null tree
  // means we allocate an empty one and leave it un-finalized. In either case,
  // we consider a non-null tree to be complete and finalize it if it hasn't
  // already been finalized.
  MultibodyTreeSystem(systems::SystemScalarConverter converter,
                      bool null_tree_is_ok,
                      std::unique_ptr<MultibodyTree<T>> tree,
                      bool is_discrete);

  const bool is_discrete_;

  std::unique_ptr<drake::multibody::internal::MultibodyTree<T>> tree_;

  // All MultibodyTreeSystem cache indexes are stored in cache_indexes_.
  CacheIndexes cache_indexes_;

  // Used to enforce "finalize once" restriction for protected-API users.
  bool already_finalized_{false};
};

/* Access internal tree outside of MultibodyTreeSystem. */
template <typename T>
const MultibodyTree<T>& GetInternalTree(const MultibodyTreeSystem<T>& system) {
  return system.internal_tree();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

namespace drake {
namespace multibody {
// Forward delcaration of MultibodyElement for attorney-client.
template <template <typename> class ElementType, typename T,
          typename ElementIndexType>
class MultibodyElement;

namespace internal {

// Attorney to give access to MultibodyElement to a selection of protected
// methods for declaring/accessing/mutating MultibodyTreeSystem parameters,
template <typename T>
class MultibodyTreeSystemElementAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeSystemElementAttorney);
  MultibodyTreeSystemElementAttorney() = delete;

 private:
  template <template <typename> class ElementType, typename U,
            typename ElementIndexType>
  friend class drake::multibody::MultibodyElement;

  static systems::NumericParameterIndex DeclareNumericParameter(
      MultibodyTreeSystem<T>* tree_system,
      const systems::BasicVector<T>& model_vector) {
    return systems::NumericParameterIndex{
        tree_system->DeclareNumericParameter(model_vector)};
  }

  static systems::AbstractParameterIndex DeclareAbstractParameter(
      MultibodyTreeSystem<T>* tree_system, const AbstractValue& model_value) {
    return systems::AbstractParameterIndex{
        tree_system->DeclareAbstractParameter(model_value)};
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::MultibodyTreeSystem)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::MultibodyTreeSystemElementAttorney)
