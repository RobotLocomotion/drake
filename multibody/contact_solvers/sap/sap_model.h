#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_bundle.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/supernodal_solver.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Enum to specify the type of the factorization to be used by SAP.
enum class SapHessianFactorizationType {
  // Block sparse supernodal solver implemented by BlockSparseCholeskySolver.
  kBlockSparseCholesky,
  // Dense algebra. Typically used for testing.
  kDense,
};

// N.B. Ideally we'd like to nest the caching structs below into SapModel.
// The structs below used for caching computations are stored by our systems::
// framework as `Value<{}>` types. Value<{}> suffers from impaired performance
// if these structs are nested within a templated class such as SapModel. To
// avoid this issue, we opt for not using nested definitions.

// Struct used to store SapConstraintBundleData in a cache entry.
// It is made cloneable to provide drake::Value semantics.
struct SapConstraintBundleDataCache {
  // Copy semantics not allowed.
  SapConstraintBundleDataCache(const SapConstraintBundleDataCache&) = delete;
  void operator=(const SapConstraintBundleDataCache&) = delete;

  // Move semantics allowed.
  SapConstraintBundleDataCache(SapConstraintBundleDataCache&&) = default;
  SapConstraintBundleDataCache& operator=(SapConstraintBundleDataCache&&) =
      default;
  SapConstraintBundleDataCache() = default;

  SapConstraintBundleData bundle_data;

  // Returns a deep-copy of `this` cache data.
  std::unique_ptr<SapConstraintBundleDataCache> Clone() const {
    auto clone = std::make_unique<SapConstraintBundleDataCache>();
    clone->bundle_data.reserve(bundle_data.size());
    for (const auto& data : bundle_data) {
      clone->bundle_data.emplace_back(data->Clone());
    }
    return clone;
  }
};

// Struct used to store the the result of updating impulses.
template <typename T>
struct ImpulsesCache {
  void Resize(int num_constraint_equations) {
    gamma.resize(num_constraint_equations);
  }
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
  T momentum_cost{NAN};     // Momentum cost, = 1/2⋅(v−v*)ᵀ⋅A⋅(v−v*).
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

// This class stores the factorization of the Hessian matrix as a
// SuperNodalSolver, with the underlying type specified by
// SapHessianFactorizationType.
class HessianFactorizationCache {
 public:
  /* We allow move semantics and forbid (expensive) copy semantics. */
  HessianFactorizationCache(const HessianFactorizationCache&) = delete;
  HessianFactorizationCache& operator=(const HessianFactorizationCache&) =
      delete;
  HessianFactorizationCache(HessianFactorizationCache&&) = default;
  HessianFactorizationCache& operator=(HessianFactorizationCache&&) = default;

  // Creates an empty cache, with no heap allocation. This is used to delay the
  // creation of the expensive Hessian structure to when it is strictly needed.
  // After instantiations with this constructor is_empty() is `true`.
  HessianFactorizationCache() = default;

  // Constructor for a cache entry that stores the factorization of a SAP
  // Hessian.
  // This class can hold references to A and J and therefore they must outlive
  // an instance of this class.
  //
  // @note is_empty() will be `false` after construction with this constructor.
  //
  // @warning This is a potentially expensive constructor, performing the
  // necessary symbolic analysis for the case of sparse factorizations.
  //
  // @pre A and J are not nullptr.
  HessianFactorizationCache(SapHessianFactorizationType type,
                            const std::vector<MatrixX<double>>* A,
                            const BlockSparseMatrix<double>* J);

  // @returns `true` if `this` factorization was never provided with a type and
  // matrices A and J.
  bool is_empty() const { return factorization_ == nullptr; }

  // Returns pointer to the underlying factorization. nullptr iff is_empty().
  const SuperNodalSolver* factorization() const { return factorization_.get(); }

  // Mutable pointer to the underlying factorization. nullptr iff is_empty().
  SuperNodalSolver* mutable_factorization() { return factorization_.get(); }

  // Updates the weight matrix G in H = A + Jᵀ⋅G⋅J and the factorization of H.
  // @pre is_empty() is false.
  void UpdateWeightMatrixAndFactor(const std::vector<MatrixX<double>>& G);

  // Solves the system x = H⁻¹⋅b in place. That is, b must store the right hand
  // side on input and it will store x on output.
  // This signature can solve a system with multiple right hand sides, stored as
  // columns of b.
  // @pre is_empty() is false and UpdateWeightMatrixAndFactor() was already
  // called.
  void SolveInPlace(EigenPtr<MatrixX<double>> b) const;

  // This Clone() implementation is provided to satisfy the "drake::Value"
  // semantics needed to place this object in a cache entry for SAP. We know
  // however that cloning this object is an expensive operation that must be
  // avoided at all costs. Therefore this implementation always throws. We can
  // do this because in the well controlled lifespan of a context generated by a
  // SapModel, we know that a HessianFactorizationCache is default constructed
  // and a Clone() method is never invoked.
  std::unique_ptr<HessianFactorizationCache> Clone() const;

 private:
  std::unique_ptr<SuperNodalSolver> factorization_;
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
  explicit SapModel(const SapContactProblem<T>* problem,
                    SapHessianFactorizationType hessian_type =
                        SapHessianFactorizationType::kBlockSparseCholesky);

  /* Returns a reference to the contact problem being modeled by this class. */
  const SapContactProblem<T>& problem() const {
    DRAKE_ASSERT(problem_ != nullptr);
    return *problem_;
  }

  /* Returns the type of factorization used for the Hessian. */
  SapHessianFactorizationType hessian_type() const { return hessian_type_; }

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

  /* Inverse of the diagonal matrix formed with the square root of the diagonal
   entries of the momentum matrix, i.e. diag(A)^{-1/2}, of size
   num_velocities(). This matrix is used to compute a scaled momentum
   residual, see [Castro et al. 2022]. */
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

  const SapConstraintBundleData& EvalSapConstraintBundleData(
      const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().bundle_data)
        .template Eval<SapConstraintBundleDataCache>(context)
        .bundle_data;
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

  const T& EvalConstraintsCost(const systems::Context<T>& context) const {
    return system_->get_cache_entry(system_->cache_indexes().cost)
        .template Eval<CostCache<T>>(context)
        .regularizer_cost;
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

  /* This method evaluates the Hessian H of the primal cost ℓₚ(v).

   @note This is an expensive computation and therefore only T = double is
   supported. When working with T = AutoDiffXd there are much more efficient
   and accurate ways to propagate gradients (namely implicit function theorem)
   than by a simple brute force propagation of gradients through the
   factorization itself. Therefore we discourage this practice by only
   providing Hessian support for T = double.

   @throws std::runtime_error when attempting to perform the computation on T
   != double. */
  const HessianFactorizationCache& EvalHessianFactorizationCache(
      const systems::Context<T>& context) const {
    return system_
        ->get_cache_entry(system_->cache_indexes().hessian_factorization)
        .template Eval<HessianFactorizationCache>(context);
  }

 private:
  /* Facilitate testing. */
  friend class SapModelTester;

  /* System used to manage context resources. */
  class SapModelSystem : public systems::LeafSystem<T> {
   public:
    /* Struct used to conglomerate the indexes of all cache entries declared by
     the model. */
    struct CacheIndexes {
      systems::CacheIndex constraint_velocities;
      systems::CacheIndex bundle_data;
      systems::CacheIndex cost;
      systems::CacheIndex gradients;
      systems::CacheIndex hessian;
      systems::CacheIndex hessian_factorization;
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

  /* SapModel builds a model at construction that is meant to remain const for
   its lifetime. We place all this model data into this structure as a way to
   properly document its intent to future developers. */
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

  /* Computes a diagonal approximation of the Delassus operator used for
   regularization of the contact problem leading to better numerical
   conditioning.

   The output of this method is later consumed by
   SapConstraintBundle<T>::MakeData(), and therefore delassus_diagonal has size
   num_constraint_equations() and its entries correspond to constraint equations
   in cluster order as specified by the contact problem's graph. Refer to the
   class documentation for ContactProblemGraph for a definition of cluster and
   their ordering in the graph. See also SapContactProblem::graph(),
   ContactProblemGraph::clusters().

   Given an approximation Wᵢᵢ of the r×r diagonal block of the Delassus operator
   corresponding to the r-equations in the i-th constraint (in cluster order),
   the scaling for this constraint is the vector (one element per equation)
   𝙚ᵣ⋅‖Wᵢᵢ‖ᵣₘₛ, with ‖Wᵢᵢ‖ᵣₘₛ = ‖Wᵢᵢ‖/nᵢ, 𝙚ᵣ = [1, 1, …, 1]ᵀ ∈ ℝʳ. See [Castro
   et al. 2022] for details.

   @param[in]  A linear dynamics matrix for each participating clique in the
   model.
   @param[out] delassus_diagonal On output an array of size
   num_constraint_equations()

   @pre delassus_diagonal is not nullptr.
   @pre A.size() equals num_cliques().
   @pre Matrix entries stored in `A` are SPD. */
  void CalcDelassusDiagonalApproximation(const std::vector<MatrixX<T>>& A,
                                         VectorX<T>* delassus_diagonal) const;

  /* Calc methods to update cache entries. See the documentation for the
  corresponding Eval methods for details. */
  void CalcConstraintVelocities(const systems::Context<T>& context,
                                VectorX<T>* vc) const;
  void CalcConstraintBundleDataCache(
      const systems::Context<T>& context,
      SapConstraintBundleDataCache* bundle_data) const;
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
  void CalcHessianFactorizationCache(const systems::Context<T>& context,
                                     HessianFactorizationCache* hessian) const;

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
  SapHessianFactorizationType hessian_type_{
      SapHessianFactorizationType::kBlockSparseCholesky};

  /* TODO(amcastro-tri): Data below is heap allocated once per time step.
   Consider how to pre-allocate once to minimize heap allocation.
   N.B. For developers, this model data is set once at construction and is
   meant to remain const for the lifetime of the SapModel object. The name of
   the struct and the name of the data variable should be enough for
   developers to think twice before mutating any of this.
   Future efforts to mutate this data (for instance to reuse the model) must
   move data out of this struct and document properly its mutability. */
  ConstModelData const_model_data_;

  // System used to manage context resources.
  std::unique_ptr<SapModelSystem> system_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
