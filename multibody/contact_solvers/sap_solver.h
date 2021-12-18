#pragma once

#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// SAP solver parameters such as tolerances, maximum number of iterations and
// regularization parameters.
struct SapSolverParameters {
  // Stopping Criteria:
  //   SAP uses two stopping criteria, one on the optimality condition and a
  // second one on the cost, see specifics below for each criteria. SAP
  // terminates with a success status whenever one of these criteria is
  // satisfied first. The criteria on the cost is meant to detect when the
  // solver stalls due to round-off errors. When this happens, the solver
  // terminates and returns the best solution it could find with finite
  // precision arithmetic. The solver cannot do better pass this point. Under
  // normal circumstances, for a well conditioned problem, nominal values
  // documented below are designed so that the solver reaches the optimality
  // condition first. With the values documented below, it is expected that the
  // solver meets the cost condition first only when the problem is
  // ill-conditioned or when the user requests a tight tolerance below the
  // recommended nominal values. Most often SAP returns a solution to a very
  // tight accuracy, with an error significantly below the requested tolerance
  // and often close to machine precision [Castro et al., 2021]. The cost
  // criterion is meant for rare events at which the state of the multibody
  // system leads to an ill-conditioned problem for which reducing the error
  // below the specified tolerance is not possible in practice due to round-off
  // errors.

  // Optimality condition criterion: We monitor the optimality condition (for
  // SAP, balance of momentum), i.e. ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖jc‖),
  // where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum balance residual, p = A⋅v and jc =
  // Jᵀ⋅γ. The norms above are defined as ‖x‖ = ‖D⋅x‖₂, where D = diag(A)^(1/2).
  // If p is a generalized momentum, then D⋅p is a scaled generalized momentum
  // where each component has the same units, square root of Joules. Therefore
  // the norms above are used to weigh all components of the generalized
  // momentum equally.
  //
  // Nominal values:
  //  * For rel_tolerance > 1.0e-5 the solver will have no problem reaching this
  //    condition.
  //  * At rel_tolerance ≈ 1.0e-6, the solver might reach the cost condition
  //    (below) first due to round-off errors, but mostly for ill conditioned
  //    problems.
  //  * For rel_tolerance ≲ 1.0e-7 the solver will most likely reach the cost
  //    condition first.
  //
  // SolverStats::optimality_condition_reached indicates if this condition was
  // reached.
  double abs_tolerance{1.e-14};  // Absolute tolerance εₐ, square root of Joule.
  double rel_tolerance{1.e-6};   // Relative tolerance εᵣ.

  // Cost condition criterion: We monitor the decrease of the cost on each
  // iteration. It is not worth it to keep iterating if round-off errors do not
  // allow the cost to keep decreasing. Therefore SAP stops the Newton iteration
  // when the optimality condition OR the cost condition is satisfied. Given the
  // costs ℓᵐ and ℓᵐ⁺¹ at Newton iterations m and m+1 respectively, the cost
  // condition is: |ℓᵐ⁺¹−ℓᵐ| < εₐ + εᵣ (ℓᵐ⁺¹+ℓᵐ)/2.
  //
  // Interaction with the optimality condition (abs_tolerance, rel_tolerance):
  // The purpose of this condition is to detect when the solver reaches the best
  // solution within round-off errors. We expect the optimality condition to be
  // satisfied first for values of the optimality tolerances within nominal
  // values. However, we expect to reach the cost condition when:
  //  1. Optimality condition tolerances are set below nominal values.
  //  2. Ill conditioning of the problem makes reaching the optimality condition
  //     difficult, specially when in the lower range of nominal values.
  //
  // SolverStats::cost_condition_reached indicates if this condition was
  // reached.
  //
  // Nominal values:
  //  * cost_rel_tolerance: we want to detect when changes in the cost are close
  //    to machine epsilon. Since round-off errors are typically larger than
  //    machine epsilon, we typically use larger values.
  //  * cost_abs_tolerance: This has units of Joule. Here we use a value that is
  //    a few orders of magnitude smaller than abs_tolerance (with units of
  //    square root of Joule) squared.
  double cost_abs_tolerance{1.e-30};  // Absolute tolerance εₐ, in Joules.
  double cost_rel_tolerance{1.e-15};  // Relative tolerance εᵣ.
  int max_iterations{100};  // Maximum number of Newton iterations.

  // Line-search parameters.
  double ls_alpha_max{1.5};   // Maximum line search parameter allowed.
  int ls_max_iterations{40};  // Maximum number of line search iterations.
  double ls_c{1.0e-4};        // Armijo's criterion parameter.
  double ls_rho{0.8};         // Backtracking search parameter.

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency ωₙ
  // is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
  // corresponds to a diagonal approximation of the Delassuss operator for each
  // contact. See [Castro et al., 2021. §IX.A] for details.
  double beta{1.0};

  // Dimensionless parameterization of the regularization of friction. An
  // approximation for the bound on the slip velocity is vₛ ≈ σ⋅δt⋅g.
  double sigma{1.0e-3};

  // Tolerance used in impulse soft norms. In Ns.
  double soft_tolerance{1.0e-7};
};

// This class implements the Semi-Analytic Primal (SAP) solver described in
// [Castro et al., 2021].
//
// SAP uses the convex approximation of contact constraints by [Anitescu, 2006],
// with constraint regularization and analytical inverse dynamics introduced by
// [Todorov, 2014]. However SAP introduces a primal formulation in velocities
// instead of in impulses as done in previous work. This leads to a numerical
// scheme that warm-starts very effectively using velocities from the previous
// time step. In addition, SAP uses regularization to model physical compliance
// rather than to introduce constraint stabilization as previously done by
// [Todorov, 2014]. Please refer to [Castro et al., 2021] for details.
//
// - [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
//   Unconstrained Convex Formulation of Compliant Contact. Available online at
//   https://arxiv.org/abs/2110.10107.
// - [Anitescu, 2006] Anitescu M., 2006. Optimization-based simulation of
//   nonsmooth rigid multibody dynamics. Mathematical Programming, 105(1),
//   pp.113-143.
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
//   dynamics with contacts and constraints: Theory and implementation in
//   MuJoCo. In 2014 IEEE International Conference on Robotics and Automation
//   (ICRA) (pp. 6054-6061). IEEE.
//
// TODO(amcastro-tri): enable AutoDiffXd support, if only for dense matrices so
// that we can test the long term performant solution.
// @tparam_double_only
template <typename T>
class SapSolver final : public ContactSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapSolver);

  // Struct used to store statistics for each solve by SolveWithGuess().
  struct SolverStats {
    // Initializes counters and time statistics to zero.
    void Reset() {
      num_iters = 0;
      num_line_search_iters = 0;
      num_impulses_cache_updates = 0;
      num_gradients_cache_updates = 0;
    }
    int num_iters{0};              // Number of Newton iterations.
    int num_line_search_iters{0};  // Total number of line search iterations.

    // Number of impulse updates. This also includes dP/dy updates, when
    // gradients are updated.
    int num_impulses_cache_updates{0};

    // Number of times the gradients cache is updated.
    int num_gradients_cache_updates{0};

    // Indicates if the optimality condition was reached.
    bool optimality_criterion_reached{false};

    // Indicates if the cost condition was reached.
    bool cost_criterion_reached{false};
  };

  SapSolver() = default;
  ~SapSolver() final = default;

  // Solve the contact problem specified by the input data. See
  // ContactSolver::SolveWithGuess() for details. Currently, only `T = double`
  // is supported. An exception is thrown if `T != double`.
  //
  // @pre dynamics_data must contain data for inverse dynamics, i.e.
  // dynamics_data.has_inverse_dynamics() is true.
  // @pre contact_data Must contain a non-zero number of contact constraints.
  //
  // Convergence of the solver is controlled by set_parameters(). Refer to
  // SapSolverParameters for details on the convergence conditions.
  //
  // N.B. SolveWithGuess() is a non-const method and thefore changes to the
  // state of the SapSolver object are allowed. This means that when using this
  // solver in MultibodyPlant (or a DiscreteUpdateManager), it must either be
  // instantiated locally or stored within a Context cache entry to ensure
  // thread safety.
  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* result) final;

  // New parameters will affect the next call to SolveWithGuess().
  void set_parameters(const SapSolverParameters& parameters) {
    parameters_ = parameters;
  }

  // Returns solver statistics from the last call to SolveWithGuess().
  // Statistics are reset with SolverStats::Reset() on each new call to
  // SolveWithGuess().
  const SolverStats& get_statistics() const {
    return stats_;
  }

 private:
  friend class SapSolverTester;

  // This class is used to store quantities that are a function of the
  // generalized velocities in the State of the solver. This class does not
  // provide a generic caching mechanism, but the dependencies are harcoded into
  // the implementation. These dependencies are sketched below:
  //
  //      ┌───┐   ┌──┐   ┌────────┐   ┌───┐   ┌────┐   ┌────────────────┐
  //      │ v │◄──┤vc│◄──┤Impulses│◄──┤Mom│◄──┤Grad│◄──┤Search Direction│
  //      └───┘   └──┘   └────────┘   └───┘   └────┘   └────────────────┘
  //                                    ▲
  //                                    │     ┌────┐
  //                                    └─────┤Cost│
  //                                          └────┘
  // Entries in the schematic correspond to cache entries with type:
  //  - vc: VelocitiesCache
  //  - Impulses: ImpulsesCache
  //  - Mom: MomentumCache
  //  - Grad: GradientsCache
  //  - Search Direction: SearchDirectionCache
  //
  // The SapSolver class provides methods to "evaluate" these cache entries and
  // always return an up-to-date reference (the computation is performed only if
  // the cache entry is not up-to-date.) As an example, consider the evaluation
  // of the constraints velocity vc. This is accomplished with:
  //   const VectorX<T>& vc = EvalVelocitiesCache(state).vc;
  // Notice that the call site does not mention the cache directly but it uses
  // the corresponding Eval method.
  // An eval method will always return a reference to an up-to-date cache entry.
  // When a cache entry is not up-to-date, then the eval method will perform the
  // necessary computation to update it. When this update computation takes
  // place, downstream cache entries are invalidated in accordance to the
  // diagram above. In the example above, calling EvalVelocitiesCache() on a
  // valid velocities cache entry simply returns a reference to that entry. When
  // the entry is invalid, stored values are updated (computed), downstream
  // dependents marked invalid, the updated entry is marked valid, and finally a
  // reference to the updated entry is returned. Mutating the state's velocity
  // with State::mutable_v() will invalidate all cache entries.
  //
  // N.B. The schematic above must be kept in sync with the implementation.
  //
  // For mathematical quantities, we attempt follow the notation introduced in
  // [Castro et al., 2021] as best as we can, though with ASCII and Unicode
  // characters.
  class Cache {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache);
    Cache() = default;

    // Each of these cache entries must know who depends on it and must make
    // sure those get invalidated when it does.

    struct VelocitiesCache {
      void Resize(int nc) { vc.resize(3 * nc); }
      void Invalidate(Cache* cache) {
        valid = false;
        cache->mutable_impulses_cache();
      }
      bool valid{false};
      VectorX<T> vc;  // constraint velocities vc = J⋅v.
    };

    struct ImpulsesCache {
      void Resize(int nc) {
        y.resize(3 * nc);
        gamma.resize(3 * nc);
      }
      void Invalidate(Cache* cache) {
        valid = false;
        cache->mutable_momentum_cache();
      }
      bool valid{false};
      VectorX<T> y;      // The (unprojected) impulse y = −R⁻¹⋅(vc − v̂).
      VectorX<T> gamma;  // Impulse γ = P(y), with P(y) the projection operator.
    };

    struct MomentumCache {
      void Resize(int nv) {
        p.resize(nv);
        jc.resize(nv);
        momentum_change.resize(nv);
      }
      void Invalidate(Cache* cache) {
        valid = false;
        cache->mutable_cost_cache();
        cache->mutable_gradients_cache();
      }
      bool valid{false};
      VectorX<T> p;                // Generalized momentum p = A⋅v
      VectorX<T> jc;               // Generalized impulse jc = Jᵀ⋅γ
      VectorX<T> momentum_change;  // = A⋅(v−v*)
    };

    struct CostCache {
      void Invalidate(Cache*) { valid = false; }
      bool valid{false};
      T ell{NAN};   // Total primal cost, = ellA + ellR.
      T ellA{NAN};  // Velocities cost, = 1/2⋅(v−v*)ᵀ⋅A⋅(v−v*).
      T ellR{NAN};  // Regularizer cost, = 1/2⋅γᵀ⋅R⋅γ.
    };

    struct GradientsCache {
      void Resize(int nv, int nc) {
        ell_grad_v.resize(nv);
        dPdy.resize(nc);
        G.resize(nc, Matrix3<T>::Zero());
      }
      void Invalidate(Cache* cache) {
        valid = false;
        cache->mutable_search_direction_cache();
      }
      bool valid{false};
      VectorX<T> ell_grad_v;         // Gradient of the cost in v.
      std::vector<Matrix3<T>> dPdy;  // Gradient of the projection, ∂P/∂y.
      std::vector<MatrixX<T>> G;     // G = -∂γ/∂vc = dP/dy⋅R⁻¹.
    };

    struct SearchDirectionCache {
      void Resize(int nv, int nc) {
        dv.resize(nv);
        dp.resize(nv);
        dvc.resize(3 * nc);
        d2ellA_dalpha2 = NAN;
      }
      void Invalidate(Cache*) { valid = false; }
      bool valid{false};
      VectorX<T> dv;          // Search direction.
      VectorX<T> dp;          // Momentum update Δp = A⋅Δv.
      VectorX<T> dvc;         // Constraints velocities update, Δvc=J⋅Δv.
      T d2ellA_dalpha2{NAN};  // d²ellA/dα² = Δvᵀ⋅A⋅Δv.
    };

    // Resizes all the cache entries to store quantities for a problem with nv
    // generalized velocities and nc contact constraints. All existing
    // data is lost.
    void Resize(int nv, int nc);

    // Marks the cache as invalid. This is meant to be called by State when
    // mutating its internal state.
    void Invalidate() {
      velocities_cache_.Invalidate(this);
    }

   private:
    friend class SapSolver<T>;

    // Dangerous methods for const access of cache entries; you must check the
    // valid bit before using. These methods are meant to be used only within
    // SapSolver's Eval methods. Cache entries must be accessed through the
    // corresponding Eval.
    const VelocitiesCache& velocities_cache() const {
      return velocities_cache_;
    }
    const ImpulsesCache& impulses_cache() const { return impulses_cache_; }
    const MomentumCache& momentum_cache() const { return momentum_cache_; }
    const CostCache& cost_cache() const { return cost_cache_; }
    const GradientsCache& gradients_cache() const { return gradients_cache_; }
    const SearchDirectionCache& search_direction_cache() const {
      return search_direction_cache_;
    }

    // Methods for mutable access of cache entries. The specific cache entry and
    // its dependents are marked invalid before it is returned. External use of
    // these methods must be only within SapSolver's Eval methods. (Internally
    // we use them for the invalidation side effect.)
    VelocitiesCache& mutable_velocities_cache() {
      velocities_cache_.Invalidate(this);
      return velocities_cache_;
    }
    ImpulsesCache& mutable_impulses_cache() {
      impulses_cache_.Invalidate(this);
      return impulses_cache_;
    }
    MomentumCache& mutable_momentum_cache() {
      momentum_cache_.Invalidate(this);
      return momentum_cache_;
    }
    CostCache& mutable_cost_cache() {
      cost_cache_.Invalidate(this);
      return cost_cache_;
    }
    GradientsCache& mutable_gradients_cache() {
      gradients_cache_.Invalidate(this);
      return gradients_cache_;
    }
    SearchDirectionCache& mutable_search_direction_cache() {
      search_direction_cache_.Invalidate(this);
      return search_direction_cache_;
    }

    // SapSolver must never access these members directly.
    VelocitiesCache velocities_cache_;
    ImpulsesCache impulses_cache_;
    MomentumCache momentum_cache_;
    CostCache cost_cache_;
    GradientsCache gradients_cache_;
    SearchDirectionCache search_direction_cache_;
  };

  // Everything in this solver is a function of the generalized velocities v;
  // cost ℓ(v), gradient ∇ℓ(v), Hessian H(v), intermediate quantities used for
  // their computation and even search direction Δv(v) and Hessian
  // factorization. State stores generalized velocities v and cached quantities
  // that are functions of v. The purpose of cached quantities is twofold; 1) to
  // allow reusing already computed quantities and 2) to centralize and
  // minimize (costly) heap allocations.
  class State {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    State() = default;

    // Constructs a state for a problem with nv generalized velocities and nc
    // contact constraints.
    State(int nv, int nc) { Resize(nv, nc); }

    // Resizes the state for a problem with nv generalized velocities and nc
    // contact constraints.
    void Resize(int nv, int nc) {
      v_.resize(nv);
      cache_.Resize(nv, nc);
    }

    const VectorX<T>& v() const { return v_; }

    VectorX<T>& mutable_v() {
      // Mark all cache quantities as invalid since they all are a function of
      // velocity.
      cache_.Invalidate();
      return v_;
    }

    const Cache& cache() const { return cache_; }

    // The state of the solver is fully described by the generalized velocities
    // v. Therefore mutating the cache does not change the state of the solver
    // and we mark this method "const". Mutable access to the cache is provided
    // to allow updating expensive quantities that are reused in multiple
    // places.
    Cache& mutable_cache() const { return cache_; }

   private:
    VectorX<T> v_;
    // N.B. State objects are only created within the scope of SolveWithGuess()
    // and therefore the implementation is thread safe. That is, SapSolver does
    // not have State members.
    mutable Cache cache_;
  };

  // Structure used to store input data pre-processed for computation. For
  // mathematical quantities, we attempt to follow the notation introduced in
  // [Castro et al., 2021] as best as we can, though with ASCII and Unicode
  // symbols.
  struct PreProcessedData {
    // Constructs an empty data.
    PreProcessedData() = default;

    // @param nv Number of generalized velocities.
    // @param nc Number of contact constraints.
    // @param dt The discrete time step used for simulation.
    PreProcessedData(double dt, int nv_in, int nc_in) : time_step(dt) {
      Resize(nv_in, nc_in);
    }

    // Resizes this PreProcessedData to store data for a problem with nv_in
    // generalized velocities and nc_in contact constraints. A call to this
    // method causes loss of all previously existing data.
    // @param nv_in Number of generalized velocities.
    // @param nc_in Number of contact constraints.
    void Resize(int nv_in, int nc_in) {
      nv = nv_in;
      nc = nc_in;
      const int nc3 = 3 * nc;
      R.resize(nc3);
      Rinv.resize(nc3);
      vhat.resize(nc3);
      mu.resize(nc);
      inv_sqrt_A.resize(nv);
      v_star.resize(nv);
      p_star.resize(nv);
      delassus_diagonal.resize(nc);
    }

    T time_step{NAN};        // Discrete time step used by the solver.
    int nv{0};               // Number of generalized velocities.
    int nc{0};               // Number of contacts.
    VectorX<T> R;            // (Diagonal) Regularization matrix, of size 3nc.
    VectorX<T> Rinv;         // Inverse of regularization matrix, of size 3nc.
    VectorX<T> vhat;         // Constraints stabilization velocity, of size 3nc.
    VectorX<T> mu;           // Friction coefficients, of size nc.
    BlockSparseMatrix<T> J;  // Jacobian as block-sparse matrix.
    BlockSparseMatrix<T> A;  // Momentum matrix as block-sparse matrix.
    std::vector<MatrixX<T>> At;  // Per-tree blocks of the momentum matrix.

    // Inverse of the diagonal matrix formed with the square root of the
    // diagonal entries of the momentum matrix, i.e. inv_sqrt_A =
    // diag(A)^{-1/2}. This matrix is used to compute a scaled momentum
    // residual, see discussion on tolerances in SapSolverParameters for
    // details.
    VectorX<T> inv_sqrt_A;

    VectorX<T> v_star;  // Free-motion generalized velocities.
    VectorX<T> p_star;  // Free motion generalized impulse, i.e. p* = M⋅v*.
    VectorX<T> delassus_diagonal;  // Delassus operator diagonal approximation.
  };

  // Computes a diagonal approximation of the Delassus operator used to compute
  // a per constraint diagonal scaling into delassus_diagonal. Given an
  // approximation Wₖₖ of the block diagonal element corresponding to the k-th
  // constraint, the scaling is computed as delassus_diagonal[k] = ‖Wₖₖ‖ᵣₘₛ =
  // ‖Wₖₖ‖/3. See [Castro et al. 2021] for details.
  // @pre Matrix entries stored in `At` are SPD.
  void CalcDelassusDiagonalApproximation(int nc,
                                         const std::vector<MatrixX<T>>& At,
                                         const BlockSparseMatrix<T>& Jblock,
                                         VectorX<T>* delassus_diagonal) const;

  // This method extracts and pre-processes input data into a format that is
  // more convenient for computation. In particular, it computes quantities
  // directly appearing in the optimization problem such as R, v̂, W, among
  // others.
  void PreProcessData(const T& time_step,
                      const SystemDynamicsData<T>& dynamics_data,
                      const PointContactData<T>& contact_data,
                      PreProcessedData* data) const;

  // Computes gamma = P(y) where P(y) is the projection of y onto the friction
  // cone defined by `mu` using the norm defined by `R`. The gradient dP/dy of
  // the operator is computed if dPdy != nullptr.
  // See [Castro et al., 2021] for details on the projection operator (Section
  // VII) and its gradients (Appendix E).
  //
  // @pre R has the form R = (Rt, Rt, Rn).
  Vector3<T> ProjectSingleImpulse(
      const T& mu, const Eigen::Ref<const Vector3<T>>& R,
      const Eigen::Ref<const Vector3<T>>& y, Matrix3<T>* dPdy = nullptr) const;

  // Computes the projection gamma = P(y) for all impulses.
  // @pre gamma must already be properly sized.
  void ProjectAllImpulses(const VectorX<T>& y, VectorX<T>* gamma) const;

  // Computes the gradient dP/dy of the projection gamma = P(y) computed by
  // ProjectAllImpulses().
  // @pre dPdy must already be properly sized.
  // TODO(amcastro-tri): consider caching common terms computed by
  // ProjectAllImpulses() so that they can be re-used by this method.
  void CalcAllProjectionsGradients(const VectorX<T>& y,
                                   std::vector<Matrix3<T>>* dPdy) const;

  // Pack solution into ContactSolverResults. Where v is the vector of
  // generalized velocities, vc is the vector of contact velocities and gamma is
  // the vector of generalized contact impulses.
  static void PackContactResults(const PreProcessedData& data,
                                 const VectorX<T>& v, const VectorX<T>& vc,
                                 const VectorX<T>& gamma,
                                 ContactSolverResults<T>* result);

  // We monitor the optimality condition (for SAP, balance of momentum), i.e.
  // ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖j‖), where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum
  // balance residual, p = A⋅v and j = Jᵀ⋅γ. The norms above are weighted as ‖x‖
  // = ‖D⋅x‖₂ where D = diag(A)^(1/2), such that all generalized momenta have
  // the same units (square root of Joules).
  // This method computes momentum_residual = ‖∇ℓ‖ and momentum_scale =
  // max(‖p‖,‖j‖). See [Castro et al., 2021] for further details.
  void CalcStoppingCriteriaResidual(const State& state, T* momentum_residual,
                                    T* momentum_scale) const;

  // Solves the contact problem from initial guess `v_guess` into `result`.
  // @pre PreProcessData() has already been called.
  ContactSolverStatus DoSolveWithGuess(const VectorX<T>& v_guess,
                                       ContactSolverResults<T>* result);

  // Computes the cost ℓ(α) = ℓ(vᵐ + αΔvᵐ) for line search, where vᵐ and Δvᵐ are
  // the last Newton iteration values of generalized velocities and search
  // direction, respectively. This methods uses the O(n) strategy described in
  // [Castro et al., 2021].
  T CalcLineSearchCost(const State& state_v, const T& alpha,
                       State* state_alpha) const;

  // Approximation to the 1D minimization problem α = argmin ℓ(α) = ℓ(v + αΔv)
  // over α. We define ϕ(α) = ℓ₀ + α c ℓ₀', where ℓ₀ = ℓ(0), ℓ₀' = dℓ/dα(0) and
  // c is the Armijo's criterion parameter. With this definition the Armijo
  // condition reads ℓ(α) < ϕ(α). This approximate method seeks to minimize ℓ(α)
  // over a discrete set of values given by the geometric progression αᵣ =
  // ρʳαₘₐₓ with r an integer, 0 < ρ < 1 and αₘₐₓ the maximum value of α
  // allowed. That is, the exact problem is replaced by a search over the
  // discrete values αᵣ until Armijo's criteria is satisfied. The satisfaction
  // of Armijo's criteria allows to prove the global convergence of SAP.
  // For a good reference on this method, see Section 11.3 of Bierlaire, M.,
  // 2015. "Optimization: principles and algorithms", EPFL Press.
  //
  // @returns A pair (α, num_iterations) where α satisfies Armijo's criterion
  // and num_iterations is the number of backtracking iterations performed.
  std::pair<T, int> PerformBackTrackingLineSearch(const State& state,
                                                  const VectorX<T>& dv) const;

  // Solves for dv using dense algebra, for debugging.
  // TODO(amcastro-tri): Add AutoDiffXd support.
  void CallDenseSolver(const State& s, VectorX<T>* dv) const;

  // Methods used to evaluate cached quantities.
  const typename Cache::VelocitiesCache& EvalVelocitiesCache(
      const State& state) const;
  const typename Cache::ImpulsesCache& EvalImpulsesCache(
      const State& state) const;
  const typename Cache::CostCache& EvalCostCache(const State& state) const;
  const typename Cache::MomentumCache& EvalMomentumCache(
      const State& state) const;
  const typename Cache::GradientsCache& EvalGradientsCache(
      const State& state) const;
  const typename Cache::SearchDirectionCache& EvalSearchDirectionCache(
      const State& state) const;

  SapSolverParameters parameters_;
  PreProcessedData data_;
  // Stats are mutable so we can update them from within const methods (e.g.
  // Eval() methods). Nothing in stats is allowed to affect the computation; it
  // is purely a passive observer.
  // TODO(amcastro-tri): Consider moving stats into the State.
  mutable SolverStats stats_;
};

template <>
ContactSolverStatus SapSolver<double>::DoSolveWithGuess(
    const VectorX<double>&, ContactSolverResults<double>*);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

extern template class ::drake::multibody::contact_solvers::internal::SapSolver<
    double>;
