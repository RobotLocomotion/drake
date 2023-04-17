#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_bundle.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// N.B. Ideally we'd like to nest the caching structs below into SapModel.
// The structs below used for caching computations are stored by our systems::
// framework as `Value<{}>` types. Value<{}> suffers from impaired performance
// if these structs are nested within a templated class such as SapModel. To
// avoid this issue, we opt for not using nested definitions.

// Struct used to store the the result of updating impulses.
template <typename T>
struct ImpulsesCache {
  void Resize(int num_constraint_equations) {
    y.resize(num_constraint_equations);
    gamma.resize(num_constraint_equations);
  }
  VectorX<T> y;      // The (unprojected) impulse y = −R⁻¹⋅(vc − v̂).
  VectorX<T> gamma;  // Impulse γ = P(y), with P(y) the projection operator.
};

// Struct used to store the result of updating the momentum gain A⋅(v−v*).
template <typename T>
struct MomentumGainCache {
  void Resize(int nv) {
    p.resize(nv);
    velocity_gain.resize(nv);
    momentum_gain.resize(nv);
  }
  VectorX<T> p;              // = A⋅v
  VectorX<T> velocity_gain;  // = v-v*
  VectorX<T> momentum_gain;  // = A⋅(v-v*)
};

// Struct used to store the cost update.
template <typename T>
struct CostCache {
  T cost{NAN};  // Total primal cost, = momentum_cost + regularizer_cost.
  T momentum_cost{NAN};  // Momentum cost, = 1/2⋅(v−v*)ᵀ⋅A⋅(v−v*).
  T regularizer_cost{NAN};  // Regularizer cost, = 1/2⋅γᵀ⋅R⋅γ.
};

// Struct used to store the result of updating the gradient of the cost.
template <typename T>
struct GradientsCache {
  void Resize(int nv) {
    j.resize(nv);
    cost_gradient.resize(nv);
  }
  VectorX<T> j;              // Generalized impulses, j = Jᵀ⋅γ.
  VectorX<T> cost_gradient;  // Gradient of the cost in v.
};

// Struct used to store the result of updating the constraint's Hessian G =
// d²ℓ/dvc² = -dγ/dvc.
template <typename T>
struct HessianCache {
  void Resize(int num_constraints, int num_constraint_equations) {
    impulses.Resize(num_constraint_equations);
    G.resize(num_constraints);
  }
  // Impulses are computed as a side effect of updating G. We keep a separate
  // copy to avoid heap allocation of auxiliary variables.
  ImpulsesCache<T> impulses;
  std::vector<MatrixX<T>> G;  // Constraint Hessian, G = -dγ/dvc = dP/dy⋅R⁻¹.
};

/* This class represents the underlying computational model built by the SAP
 solver given a SapContactProblem.
 The model re-arranges constraints to exploit the block sparse structure of the
 problem and it only includes participating cliques, that is, cliques that
 connect to a cluster (edge) in the contact graph. The solution for
 non-participating cliques is trivial (v=v*) and therefore they are excluded
 from the main (expensive) SAP computation.

 This model is meant to be constructed once per time step, it does not persist
 over multiple time steps. Given a multibody system with state x, this model is
 built for a given state x₀ at time t so that we can take a discrete time step
 of size δt and advance the state of the system to time t+δt. In other words,
 the model is a function of state x₀ and therefore it must be updated at the
 next time step.

 Our SAP solver will create an instance of a SapModel to solve a given
 SapContactProblem per discrete time step. The model remains "const" after
 creation and only (const) state dependent queries on this model can be
 performed. The state of the model is stored as a systems::Context. This Context
 is created by the SAP solver invoking the method SapModel::MakeContext(). Once
 a context is available, the solver can perform queries on this model; e.g.
 SapModel::EvalMomentumCost(context).

 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
 Unconstrained Convex Formulation of Compliant Contact. Available at
 https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapModel);

  /* Constructs a model of `problem` optimized to be used by the SAP solver.
   The input `problem` must outlive `this` model. */
  explicit SapModel(const SapContactProblem<T>* problem);

  /* Returns a reference to the contact problem being modeled by this class. */
  const SapContactProblem<T>& problem() const {
    DRAKE_ASSERT(problem_ != nullptr);
    return *problem_;
  }

  /* Returns the number of (participating) cliques. */
  int num_cliques() const;

  /* Returns the number of (participating) generalized velocities. */
  int num_velocities() const;

  int num_constraints() const;
  int num_constraint_equations() const;

  /* Returns the permutation to map back and forth between participating
   velocities in this model and the full vector of generalized velocities in the
   original contact problem. Given a vector v of generalized velocities for
   the contact problem associated with this model,
   velocities_permutation().Apply(v, &vp) writes the participating velocities in
   this model into vp. Similarly, given a vector vp of participating velocities,
   velocities_permutation().ApplyInverse(vp, &v) places participating velocities
   in the full vector of generalized velocities v, leaving non-participating
   velocities untouched. */
  const PartialPermutation& velocities_permutation() const {
    return const_model_data_.velocities_permutation;
  }

  /* Returns permutation to go back and forth between constraint equations in
   the original contact problem and constraint equations as ordered in this
   model for computational efficiency. This mapping can be used on impulses,
   constraint velocities or any other quantity indexed with constraint equations
   indexes. */
  const PartialPermutation& impulses_permutation() const {
    return const_model_data_.impulses_permutation;
  }

  /* The time step of problem(). */
  const T& time_step() const { return problem_->time_step(); }

  /* Returns the system dynamics matrix A, refer to SapContactProblem's
   documentation for how this matrix is defined. This method returns a vector of
   diagonal blocks in A for participating cliques only. Blocks in the returned
   vector appear in the order specified by the permutation of participating
   cliques in the problem's graph, see
   ContactProblemGraph::participating_cliques(). */
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  /* Returns free-motion velocities v*, Participating velocities only. */
  const VectorX<T>& v_star() const;

  /* Returns free-motion generalized momenta. Participating momenta only. */
  const VectorX<T>& p_star() const;

  // Inverse of the diagonal matrix formed with the square root of the diagonal
  // entries of the momentum matrix, i.e. diag(A)^{-1/2}, of size
  // num_velocities(). This matrix is used to compute a scaled momentum
  // residual, see [Castro et al. 2022].
  const VectorX<T>& inv_sqrt_dynamics_matrix() const;

  /* Const access to the bundle for this model. */
  const SapConstraintBundle<T>& constraints_bundle() const;

  /* Performs multiplication p = A⋅v. Only participating velocities are
   considered.
   @pre p must be a valid pointer.
   @pre both v and p must be of size num_participating_velocities(). */
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* p) const;

  /* Makes a context to be used on queries with this model. */
  std::unique_ptr<systems::Context<T>> MakeContext() const;

  /* Retrieves the generalized velocities stored in `context`. Of size
   num_velocities().
   @pre `context` is created with a call to MakeContext(). */
  const VectorX<T>& GetVelocities(const systems::Context<T>& context) const;

  /* Mutable version of GetVelocities().
   @pre `context` is created with a call to MakeContext().
   @warning Holding onto this mutable reference and modifying it repeatedly can
   lead to wrong answers from Eval methods. Use with care and when possible use
   SetVelocities() instead. */
  Eigen::VectorBlock<VectorX<T>> GetMutableVelocities(
      systems::Context<T>* context) const;

  /* Stores generalized velocities `v` in `context`.
   @pre v.size() equals num_velocities().
   @pre `context` is created with a call to MakeContext(). */
  void SetVelocities(const VectorX<T>& v, systems::Context<T>* context) const;

  /* Evaluates the constraint velocities vc = J⋅v.
   @pre `context` is created with a call to MakeContext(). */
  const VectorX<T>& EvalConstraintVelocities(
      const systems::Context<T>& context) const {
    return system_
        ->get_cache_entry(system_->cache_indexes().constraint_velocities)
        .template Eval<VectorX<T>>(context);
  }

  /* Evaluates the contact impulses γ(v) according to the analytical inverse
   dynamics, see [Castro et al. 2022].
   @pre `context` is created with a call to MakeContext(). */
  const VectorX<T>& EvalImpulses(const systems::Context<T>& context) const {
    return EvalImpulsesCache(context).gamma;
  }

  /* Evaluates the generalized impulses j = Jᵀ⋅γ, where J is the constraints'
   Jacobian and γ is the vector of impulses, see EvalImpulses(). The size of the
   generalized impulses j is num_velocities().
   @pre `context` is created with a call to MakeContext(). */
  const VectorX<T>& EvalGeneralizedImpulses(
      const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().gradients)
        .template Eval<GradientsCache<T>>(context)
        .j;
  }

  /* Evaluates the generalized momentum p = A⋅v as a function of the generalized
   velocities stored in `context`. */
  const VectorX<T>& EvalMomentum(const systems::Context<T>& context) const {
    return EvalMomentumGainCache(context).p;
  }

  /* Evaluates the momentum gain defined as momentum_gain = A⋅(v - v*).
   @pre `context` is created with a call to MakeContext(). */
  const VectorX<T>& EvalMomentumGain(const systems::Context<T>& context) const {
    return EvalMomentumGainCache(context).momentum_gain;
  }

  /* Evaluates the momentum cost defined as
   momentum_cost = 1/2⋅(v-v*)ᵀ⋅A⋅(v-v*).
   @pre `context` is created with a call to MakeContext(). */
  const T& EvalMomentumCost(const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().cost)
        .template Eval<CostCache<T>>(context)
        .momentum_cost;
  }

  /* Evaluates the primal cost ℓₚ(v) = 1/2⋅(v-v*)ᵀ⋅A⋅(v-v*) + 1/2⋅γᵀ⋅R⋅γ.
   @pre `context` is created with a call to MakeContext(). */
  const T& EvalCost(const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().cost)
        .template Eval<CostCache<T>>(context)
        .cost;
  }

  /* Evaluates the gradient w.r.t. participating generalized velocities of the
   cost.
   @pre `context` is created with a call to MakeContext(). */
  const VectorX<T>& EvalCostGradient(const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().gradients)
        .template Eval<GradientsCache<T>>(context)
        .cost_gradient;
  }

  /* Evaluates the constraints's Hessian G, a vector of size num_constraints().
   For the i-th constraint, its Hessian is defined as Gᵢ(v) = d²ℓᵢ/dvcᵢ², where
   ℓᵢ is the regularizer's cost ℓᵢ = 1/2⋅γᵢᵀ⋅Rᵢ⋅γᵢ and vcᵢ is the constraint's
   velocity. Gᵢ is an SPD matrix. It turns out that the Hessian of SAP's cost is
   H = A + Jᵀ⋅G⋅J, see [Castro et al. 2022].
   @pre `context` is created with a call to MakeContext(). */
  const std::vector<MatrixX<T>>& EvalConstraintsHessian(
      const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().hessian)
        .template Eval<HessianCache<T>>(context)
        .G;
  }

 private:
  // Facilitate testing.
  friend class SapModelTester;

  /* System used to manage context resources. */
  class SapModelSystem : public systems::LeafSystem<T> {
   public:
    /* Struct used to conglomerate the indexes of all cache entries declared by
     the model. */
    struct CacheIndexes {
      systems::CacheIndex constraint_velocities;
      systems::CacheIndex cost;
      systems::CacheIndex gradients;
      systems::CacheIndex hessian;
      systems::CacheIndex impulses;
      systems::CacheIndex momentum_gain;
    };

    /* Constructs a system that declares a discrete state of size
     num_velocities. */
    explicit SapModelSystem(int num_velocities) {
      velocities_index_ = this->DeclareDiscreteState(num_velocities);
    }

    /* Promote system methods so that SapModel can use them to declare state and
     cache entries. */
    using systems::SystemBase::DeclareCacheEntry;

    /* Index of the discrete state declared by this system. */
    systems::DiscreteStateIndex velocities_index() const {
      return velocities_index_;
    }

    /* Accessors to cache indexes. */
    const CacheIndexes& cache_indexes() const { return cache_indexes_; }
    CacheIndexes& mutable_cache_indexes() { return cache_indexes_; }

   private:
    systems::DiscreteStateIndex velocities_index_;
    CacheIndexes cache_indexes_;
  };

  // SapModel builds a model at construction that is meant to remain const for
  // its lifetime. We place all this model data into this structure as a way to
  // properly document its intent to future developers.
  struct ConstModelData {
    /* Permutation to map back and forth between DOFs in problem_ and DOFs in
     this model. */
    PartialPermutation velocities_permutation;
    /* Permutation to map back and forth between impulses in problem_ and
     impulses as ordered according to the contact graph in this model. */
    PartialPermutation impulses_permutation;
    /* Per-clique blocks of the system's dynamic matrix A. Only participating
     cliques. */
    std::vector<MatrixX<T>> dynamics_matrix;
    VectorX<T> v_star;  // Free motion generalized velocity v*.
    VectorX<T> p_star;  // Free motion generalized impulse, i.e. p* = A⋅v*.
    // Inverse of the diagonal matrix formed with the square root of the
    // diagonal entries of the momentum matrix, i.e. diag(A)^{-1/2}.
    VectorX<T> inv_sqrt_A;
    VectorX<T> delassus_diagonal;  // Delassus operator diagonal approximation.
    std::unique_ptr<SapConstraintBundle<T>> constraints_bundle;
  };

  /* N.B. on the use of systems::Context and caching to future developers.
   The SapModel class uses systems::Context to store its state and make use of
   the very well tested caching system. To accomplish this, SapModel privately
   owns a SapModelSystem used to manage this resources in the context, even
   though SapModel is NOT a systems::System. A few important notes on
   DeclareCacheEntries():
     1. This method is intended to be called ONLY from the model's constructor,
        AFTER the SapModelSystem used to manage its resources is created.
        Calling this method before the SapModelSystem is created will trigger an
        assertion failure.
     2. The Calc() and Eval() methods for these cache entries belong to
        SapModel, not to the system used to declare them. This is required by
        the fact that model data belongs to SapModel. */
  void DeclareCacheEntries();

  /* Makes a permutation to go back and forth between dofs in the original
   contact problem and "participating dofs" in this model. */
  static PartialPermutation MakeParticipatingVelocitiesPermutation(
      const SapContactProblem<T>& problem);

  /* Makes a permutation to go back and forth between constraint equations in
   the original contact problem and constraint equations as ordered in this
   model for computational efficiency. This mapping can be used on impulses,
   constraint velocities or any other quantity indexed as constraint equations.
   */
  PartialPermutation MakeImpulsesPermutation(
      const ContactProblemGraph& graph) const;

  // Computes a diagonal approximation of the Delassus operator used to compute
  // a per constraint diagonal scaling into delassus_diagonal. Given an
  // approximation Wᵢᵢ of the block diagonal element corresponding to the i-th
  // constraint, the scaling is computed as delassus_diagonal[i] = ‖Wᵢᵢ‖ᵣₘₛ =
  // ‖Wᵢᵢ‖/nᵢ, where nᵢ is the number of equations for the i-th constraint (and
  // the size of Wᵢᵢ). See [Castro et al. 2022] for details.
  //
  // @param[in]  A linear dynamics matrix for each participating clique in the
  // model.
  // @param[out] delassus_diagonal On output an array of size nc (number of
  // constraints) where each entry stores the Delassus operator constraint
  // scaling. That is, wi = delassus_diagonal[i] corresponds to the scaling for
  // the i-th constraint.
  //
  // @pre delassus_diagonal is not nullptr.
  // @pre A.size() equals num_cliques().
  // @pre Matrix entries stored in `A` are SPD.
  void CalcDelassusDiagonalApproximation(const std::vector<MatrixX<T>>& A,
                                         VectorX<T>* delassus_diagonal) const;

  /* Calc methods to update cache entries. See the documentation for the
  corresponding Eval methods for details. */
  void CalcConstraintVelocities(const systems::Context<T>& context,
                                VectorX<T>* vc) const;
  void CalcImpulsesCache(const systems::Context<T>& context,
                         ImpulsesCache<T>* cache) const;
  void CalcMomentumGainCache(const systems::Context<T>& context,
                             MomentumGainCache<T>* cache) const;
  void CalcCostCache(const systems::Context<T>& context,
                     CostCache<T>* cache) const;
  void CalcGradientsCache(const systems::Context<T>& context,
                          GradientsCache<T>* cache) const;
  void CalcHessianCache(const systems::Context<T>& context,
                        HessianCache<T>* cache) const;

  /* Evaluates the velocity gain defined as velocity_gain = v - v*. */
  const VectorX<T>& EvalVelocityGain(const systems::Context<T>& context) const {
    return EvalMomentumGainCache(context).velocity_gain;
  }

  /* Evaluates the momentum gain A⋅(v−v*). */
  const MomentumGainCache<T>& EvalMomentumGainCache(
      const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().momentum_gain)
        .template Eval<MomentumGainCache<T>>(context);
  }

  /* Helper to evaluate the impulses cache entry. */
  const ImpulsesCache<T>& EvalImpulsesCache(
      const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().impulses)
        .template Eval<ImpulsesCache<T>>(context);
  }

  const SapContactProblem<T>* problem_{nullptr};

  // TODO(amcastro-tri): Data below is heap allocated once per time step.
  // Consider how to pre-allocate once to minimize heap allocation.
  // N.B. For developers, this model data is set once at construction and is
  // meant to remain const for the lifetime of the SapModel object. The name of
  // the struct and the name of the data variable should be enough for
  // developers to think twice before mutating any of this.
  // Future efforts to mutate this data (for instance to reuse the model) must
  // move data out of this struct and document properly its mutability.
  ConstModelData const_model_data_;

  // System used to manage context resources.
  std::unique_ptr<SapModelSystem> system_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
