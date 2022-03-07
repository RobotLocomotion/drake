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

  /* Returns the number or (participating) generalized velocities.*/
  int num_velocities() const;

  int num_constraints() const;
  int num_constraint_equations() const;

  const PartialPermutation& velocities_permutation() const {
    return velocities_permutation_;
  }

  /* The time step of problem(). */
  const T& time_step() const { return problem_->time_step(); }

  /* Returns the system dynamics matrix A, refer to SapContactProblem's
   documentation for how this matrix is defined. This method returns a vector of
   diagonal blocks in A for participating cliques only. Blocks in the returned
   vector appear in the order specified by the permutation of participating
   cliques in the problem's graph, see
   SapContactProblem::graph().participating_cliques(). */
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  /* Returns free-motion velocities v*, Participating velocities only. */
  const VectorX<T>& v_star() const;

  /* Returns free-motion generalized momenta. Participating momenta only. */
  const VectorX<T>& p_star() const;

  /* Makes a context to be used on queries with this model. */
  std::unique_ptr<systems::Context<T>> MakeContext() const;

  /* Retrieves the generalized velocities stored in `context`. Of size
   num_velocities(). */
  const VectorX<T>& GetVelocities(const systems::Context<T>& context) const;

  /* Stores generalized velocities `v` in `context`.
   @throws exception if v.size() is not num_velocities(). */
  void SetVelocities(const VectorX<T>& v, systems::Context<T>* context) const;

  /* Evaluates the constraint velocities vc = J⋅v. */
  const VectorX<T>& EvalConstraintVelocities(
      const systems::Context<T>& context) const {
    return system_
        ->get_cache_entry(system_->cache_indexes().constraint_velocities)
        .template Eval<VectorX<T>>(context);
  }

  /* Evaluates the momentum gain defined as momentum_gain = A⋅(v - v*). */
  const VectorX<T>& EvalMomentumGain(const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().momentum_gain)
        .template Eval<VectorX<T>>(context);
  }

  /* Evaluates the momentum cost defined as
   momentum_cost = 1/2⋅(v-v*)ᵀ⋅A⋅(v-v*). */
  const T& EvalMomentumCost(const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().momentum_cost)
        .template Eval<T>(context);
  }

 private:
  /* System used to manage context resources. */
  class SapModelSystem : public systems::LeafSystem<T> {
   public:
    /* Struct used to conglomerate the indexes of all cache entries declared by
     the model. */
    struct CacheIndexes {
      systems::CacheIndex constraint_velocities;
      systems::CacheIndex momentum_cost;
      systems::CacheIndex momentum_gain;
      systems::CacheIndex velocity_gain;
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

  void DeclareCacheEntries();

  /* Makes a permutation to go back and forth between dofs in the original
   contact problem and "participating dofs" in this model. */
  static PartialPermutation MakeParticipatingVelocitiesPermutation(
      const SapContactProblem<T>& problem);

  /* Performs multiplication p = A * v. Only participating velocities are
   considered.
   @pre p must be a valid pointer.
   @pre both v and p must be of size num_participating_velocities(). */
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* p) const;

  /* Calc methods to update cache entries. See the documentation for the
  corresponding Eval methods for details. */
  void CalcConstraintVelocities(const systems::Context<T>& context,
                                VectorX<T>* vc) const;
  void CalcVelocityGain(const systems::Context<T>& context,
                        VectorX<T>* velocity_gain) const;
  void CalcMomentumGain(const systems::Context<T>& context,
                        VectorX<T>* momentum_gain) const;
  void CalcMomentumCost(const systems::Context<T>& context,
                        T* momentum_cost) const;

  /* Evaluates the velocity gain defined as velocity_gain = v - v*. */
  const VectorX<T>& EvalVelocityGain(const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().velocity_gain)
        .template Eval<VectorX<T>>(context);
  }

  const SapContactProblem<T>* problem_{nullptr};

  // TODO(amcastro-tri): Data below is heap allocated once per time step.
  // Consider how to pre-allocate once to minimize heap allocation.

  /* Permutation to map back and forth between DOFs in problem_ and DOFs in this
   model. */
  PartialPermutation velocities_permutation_;
  /* Per-clique blocks of the system's dynamic matrix A. Only participating
  cliques. */
  std::vector<MatrixX<T>> dynamics_matrix_;
  VectorX<T> v_star_;  // Free motion generalized velocity v*.
  VectorX<T> p_star_;  // Free motion generalized impulse, i.e. p* = A⋅v*.
  std::unique_ptr<SapConstraintBundle<T>> constraints_bundle_;

  // System used to manage context resources.
  std::unique_ptr<SapModelSystem> system_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
