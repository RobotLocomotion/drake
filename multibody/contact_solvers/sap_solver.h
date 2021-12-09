#pragma once

#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/partial_permutation.h"
#include "drake/multibody/contact_solvers/sap_contact_problem.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// SAP solver parameters such as tolerances, maximum number of iterations and
// regularization parameters.
struct SapSolverParameters {
  // Stopping criteria tolerances. We monitor the optimality condition (for
  // SAP, balance of momentum), i.e. ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖jc‖),
  // where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum balance residual, p = A⋅v and jc =
  // Jᵀ⋅γ. The norms above are defined as ‖x‖ = ‖D⋅x‖₂, where D = diag(A)^(1/2).
  // If p is a generalized momentum, then D⋅p is a scaled generalized momentum
  // where each component has the same units, square root of Joules. Therefore
  // the norms above are used to weigh all components of the generalized
  // momentum equally.
  double abs_tolerance{1.0e-6};  // Absolute tolerance εₐ, square root of Joule.
  double rel_tolerance{1.0e-6};  // Relative tolerance εᵣ.
  int max_iterations{100};       // Maximum number of Newton iterations.

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

template <typename T>
class SapConstraintsBundle {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraintsBundle);

  // We keep a reference to `problem` and its data.
  SapConstraintsBundle(
      BlockSparseMatrix<T>&& J,
      VectorX<T>&& vhat, 
      VectorX<T>&& R, 
      std::vector<std::unique_ptr<Projection<T>>>&& projections);

  int num_constraints() const;

  const BlockSparseMatrix<T>& J() const { return J_; }

  const VectorX<T>& R() const { return R_;  }

  const VectorX<T>& Rinv() const { return Rinv_;  }

  const VectorX<T>& vhat() const { return vhat_;  }

  void MultiplyByJacobian(const VectorX<T>& v, VectorX<T>* vc) const;

  void MultiplyByJacobianTranspose(const VectorX<T>& gamma, VectorX<T>* jc) const;

  void CalcUnprojectedImpulses(const VectorX<T>& vc, VectorX<T>* y) const;

  // Computes the projection gamma = P(y) for all impulses and the gradient
  // dP/dy if dgamma_dy != nullptr.
  void ProjectImpulses(const VectorX<T>& y, const VectorX<T>& R,
                       VectorX<T>* gamma,
                       std::vector<MatrixX<T>>* dPdy = nullptr) const;

  void CalcProjectImpulsesAndCalcConstraintsHessian(
      const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
      std::vector<MatrixX<T>>* G) const;

 private:
  // Jacobian for the entire bundle, with graph_.num_edges() block rows and
  // graph_.num_cliques() block columns.
  BlockSparseMatrix<T> J_;
  VectorX<T> vhat_;
  VectorX<T> R_;
  VectorX<T> Rinv_;
  // problem_ constraint references in the order dictated by the
  // ContactProblemGraph.
  std::vector<std::unique_ptr<Projection<T>>> projections_;
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
    int num_line_search_iters{0};  // Total number of line search interations.

    // Number of impulse updates. This also includes also includes dγ/dy
    // updates, when gradients are updated.
    int num_impulses_cache_updates{0};

    // Number of times the gradients cache is updated.
    int num_gradients_cache_updates{0};
  };

  SapSolver() = default;
  ~SapSolver() final = default;

  // Solve the contact problem specified by the input data. See
  // ContactSolver::SolveWithGuess() for details. Currently, only `T = double`
  // is supported. An exception is thrown if `T != double`.
  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* result) final;

  // New parameters will affect the next call to SolveWithGuess().
  void set_parameters(const SapSolverParameters& parameters) {
    non_thread_safe_data_.parameters = parameters;
  }

  // Returns solver statistics from the last call to SolveWithGuess().
  // Statistics are reset with SolverStats::Reset() on each new call to
  // SolveWithGuess().
  const SolverStats& get_statistics() const {
    return non_thread_safe_data_.stats;
  }

 private:
  friend class SapSolverTester;

  // This is not a real cache in the CS sense (i.e. there is no tracking of
  // dependencies nor automatic validity check) but in the sense that this class
  // stores computations that are function of the solver's state. It is the
  // responsability of the solver to keep these computations properly in sync.
  // This class does provide however some very basic level of checking. Const
  // access to specific cache entries demand that cache entries are valid (in
  // sync. with the state), E.g. Cache::impulses_cache(). Mutable access to
  // cache entries invalidates the cache entry, E.g.
  // Cache::mutable_impulses_cache(). With cache "validity" we mean a particular
  // cache entry is in sync with the velocities stored in State, E.g.
  // Cache::valid_impulses_cache(). For mathematical quantities, we attempt
  // follow the notation introduced in [Castro et al., 2021] as best as we can,
  // though with ASCII and Unicode characters.
  class Cache {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache);
    Cache() = default;

    struct VelocitiesCache {
      void Resize(int nk) { vc.resize(nk); }
      bool valid{false};
      VectorX<T> vc;  // constraint velocities vc = J⋅v.
    };

    struct MomentumCache {
      void Resize(int nv) {
        p.resize(nv);
        j.resize(nv);
        momentum_change.resize(nv);
      }
      bool valid{false};
      VectorX<T> p;                // Generalized momentum p = M⋅v
      VectorX<T> j;                // Generalized impulse j = Jᵀ⋅γ
      VectorX<T> momentum_change;  // = M⋅(v−v*)
    };

    struct ImpulsesCache {
      void Resize(int nk) {
        y.resize(nk);
        gamma.resize(nk);
      }
      bool valid{false};
      VectorX<T> y;  // The (unprojected) impulse y = −R⁻¹⋅(vc − v̂).
      VectorX<T> gamma;  // Impulse γ = P(y), with P(y) the projection operator.
    };

    struct GradientsCache {
      void Resize(int nv, int nc) {
        ell_grad_v.resize(nv);
        // N.B. The first time these are computed, they are allocated.
        G.resize(nc);
      }
      bool valid{false};
      VectorX<T> ell_grad_v;              // Gradient of the cost in v.
      std::vector<MatrixX<T>> G;          // G = -∂γ/∂vc = dP/dy⋅R⁻¹.
    };

    struct CostCache {
      bool valid{false};
      T ell{0.0};   // Total primal cost, = ellM + ellR.
      T ellM{0.0};  // Velocities cost, = 1/2⋅(v−v*)ᵀ⋅M⋅(v−v*).
      T ellR{0.0};  // Regularizer cost, = 1/2⋅γᵀ⋅R⋅γ.
    };

    struct SearchDirectionCache {
      void Resize(int nv, int nk) {
        dv.resize(nv);
        dp.resize(nv);
        dvc.resize(nk);
      }
      bool valid{false};
      VectorX<T> dv;     // Search direction.
      VectorX<T> dp;     // Momentum update Δp = M⋅Δv.
      VectorX<T> dvc;    // Constraints velocities update, Δvc=J⋅Δv.
      T d2ellM_dalpha2;  // d²ellM/dα² = Δvᵀ⋅M⋅Δv.
    };

    // @param nv Number of generalized velocities.
    // @param nc Number of constraints.
    // @param nk Number of constrained DOFs i.e. impulses dimension.
    void Resize(int nv, int nc, int nk) {
      velocities_cache_.Resize(nk);
      momentum_cache_.Resize(nv);
      impulses_cache_.Resize(nk);
      gradients_cache_.Resize(nv, nc);
      search_direction_cache_.Resize(nv, nk);
    }

    // Marks the cache as invalid. This is meant to be called by State when
    // mutating its internal state.
    void mark_invalid() {
      velocities_cache_.valid = false;
      momentum_cache_.valid = false;
      impulses_cache_.valid = false;
      gradients_cache_.valid = false;
      cost_cache_.valid = false;
      search_direction_cache_.valid = false;
    }

    // Methods used to check cache validity. Typically used by update methods to
    // skip computations when a cache entry is valid.
    bool valid_velocities_cache() const { return velocities_cache_.valid; }
    bool valid_momentum_cache() const { return momentum_cache_.valid; }
    bool valid_impulses_cache() const { return impulses_cache_.valid; }
    bool valid_gradients_cache() const { return gradients_cache_.valid; }
    bool valid_cost_cache() const { return cost_cache_.valid; }
    bool valid_search_direction_cache() const {
      return search_direction_cache_.valid;
    }

    // Methods for const access of cache entries. This methods demand the entry
    // to be valid, therefore providing some level of correctness guarantee.
    const VelocitiesCache& velocities_cache() const {
      DRAKE_DEMAND(velocities_cache_.valid);
      return velocities_cache_;
    }
    const MomentumCache& momentum_cache() const {
      DRAKE_DEMAND(momentum_cache_.valid);
      return momentum_cache_;
    }
    const ImpulsesCache& impulses_cache() const {
      DRAKE_DEMAND(impulses_cache_.valid);
      return impulses_cache_;
    }
    const GradientsCache& gradients_cache() const {
      DRAKE_DEMAND(gradients_cache_.valid);
      return gradients_cache_;
    }
    const CostCache& cost_cache() const {
      DRAKE_DEMAND(cost_cache_.valid);
      return cost_cache_;
    }
    const SearchDirectionCache& search_direction_cache() const {
      DRAKE_DEMAND(search_direction_cache_.valid);
      return search_direction_cache_;
    }

    // Methods for mutable access of cache entries. The specific cache entry is
    // marked invalid before it is returned.
    VelocitiesCache& mutable_velocities_cache() {
      velocities_cache_.valid = false;
      return velocities_cache_;
    }
    MomentumCache& mutable_momentum_cache() {
      momentum_cache_.valid = false;
      return momentum_cache_;
    }
    ImpulsesCache& mutable_impulses_cache() {
      impulses_cache_.valid = false;
      return impulses_cache_;
    }
    GradientsCache& mutable_gradients_cache() {
      gradients_cache_.valid = false;
      return gradients_cache_;
    }
    CostCache& mutable_cost_cache() {
      cost_cache_.valid = false;
      return cost_cache_;
    }
    SearchDirectionCache& mutable_search_direction_cache() {
      search_direction_cache_.valid = false;
      return search_direction_cache_;
    }

    // Convenient shortcuts for const access of commonly used quantities. N.B.
    // The implementation access them using const access to the specific cache
    // entry in order to provide validity checking.
    const VectorX<T>& vc() const { return velocities_cache().vc; }
    const VectorX<T>& gamma() const { return impulses_cache().gamma; }
    const VectorX<T>& momentum_change() const {
      return momentum_cache().momentum_change;
    }

   private:
    VelocitiesCache velocities_cache_;
    MomentumCache momentum_cache_;
    ImpulsesCache impulses_cache_;
    GradientsCache gradients_cache_;
    CostCache cost_cache_;
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
    // @param nv Number of generalized velocities.
    // @param nc Number of constraints.
    // @param nk Number of constrained DOFs i.e. impulses dimension.
    State(int nv, int nc, int nk) { Resize(nv, nc, nk); }

    // Resizes the state for a problem with nv generalized velocities and nc
    // contact constraints.
    // @param nv Number of generalized velocities.
    // @param nc Number of constraints.
    // @param nk Number of constrained DOFs i.e. impulses dimension.
    void Resize(int nv, int nc, int nk) {
      v_.resize(nv);
      cache_.Resize(nv, nc, nk);
    }

    const VectorX<T>& v() const { return v_; }

    VectorX<T>& mutable_v() {
      // Mark all cache quantities as invalid since they all are a function of
      // velocity.
      cache_.mark_invalid();
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
    mutable Cache cache_;
  };

  // Structure used to store input data pre-processed for computation. For
  // mathematical quantities, we attempt to follow the notation introduced in
  // [Castro et al., 2021] as best as we can, though with ASCII and Unicode
  // symbols.
  struct PreProcessedData {
    // Constructs an empty data.
    PreProcessedData() = default;

    // @param nv_in Number of generalized velocities.
    // @param nk_in Total number of constrained DOFs.
    // @param dt The discrete time step used for simulation.
    PreProcessedData(double dt, int nv_in, int nc_in, int nk_in)
        : time_step(dt) {
      Resize(nv_in, nc_in, nk_in);
    }

    // Resizes this PreProcessedData to store data for a problem with nv_in
    // generalized velocities and nc_in contact constraints. A call to this
    // method causes loss of all previously existing data.
    // @param nv_in Number of generalized velocities.
    // @param nk_in Total number of constrained DOFs.
    void Resize(int nv_in, int nc_in, int nk_in) {
      nv = nv_in;
      nc = nc_in;
      nk = nk_in;
      inv_sqrt_A.resize(nv);
      v_star.resize(nv);
      p_star.resize(nv);
      delassus_diagonal.resize(nc);
    }

    T time_step{NAN};        // Discrete time step used by the solver.
    int nv{0};               // Number of generalized velocities.
    int nc{0};               // Number of constraints.
    int nk{0};               // Number of constrained dofs. Number of impulses.
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
    
    std::unique_ptr<SapConstraintsBundle<T>> constraints_bundle;
  };

  // This class stores mutable data members making it non thread-safe.
  // We confine all these mutable members within this single struct.
  struct NonThreadSafeData {
    SapSolverParameters parameters;
    PreProcessedData data;
    SolverStats stats;
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
  PreProcessedData PreProcessData(
      const T& time_step, const SystemDynamicsData<T>& dynamics_data,
      const PointContactData<T>& contact_data) const;

  //PreProcessedData PreProcessData(const SapContactProblem<T>& problem) const;

  // Performs multiplication p = A * v.
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* p) const {
    int block_start = 0;
    for (const auto& Ab : data().At) {
      const int block_size = Ab.rows();
      p->segment(block_start, block_size) =
          Ab * v.segment(block_start, block_size);
      block_start += block_size;
    }
  }

  // Computes gamma = P(y) where P(y) is the projection of y onto the friction
  // cone defined by `mu` using the norm defined by `R`. The gradient dP/dy of
  // the operator is computed if dPdy != nullptr.
  // See [Castro et al., 2021] for details on the projection operator and its
  // gradients.
  Vector3<T> CalcProjectionOntoFrictionCone(
      const T& mu, const Eigen::Ref<const Vector3<T>>& R,
      const Eigen::Ref<const Vector3<T>>& y, Matrix3<T>* dPdy = nullptr) const;

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
  // = ‖D⋅x‖₂ where D = diag(A)*(1/2), such that all generalized momenta have
  // the same units (squared root of Joules).
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

  // Approximation to the 1D minimization problem α = argmin ℓ(α)= ℓ(v + αΔv)
  // over α. We define ϕ(α) = ℓ₀ + α c ℓ₀', where ℓ₀ = ℓ(0) and ℓ₀' = dℓ/dα(0).
  // With this definition the Armijo condition reads ℓ(α) < ϕ(α). This
  // approximate method seeks to minimize ℓ(α) over a discrete set of values
  // given by the geometric progression αᵣ = ρʳαₘₐₓ with r an integer, 0 < ρ < 1
  // and αₘₐₓ the maximum value of α allowed. That is, the exact problem is
  // replaced by a search over the discrete values αᵣ until Armijo's criteria is
  // satisfied. The satisfaction of Armijo's criteria allows to prove the
  // global convergence of SAP.
  T PerformBackTrackingLineSearch(const State& state,
                                  int* num_iterations) const;

  // Solves for dv using dense algebra, for debugging.
  // TODO(amcastro-tri): Add AutoDiffXd support.
  void CallDenseSolver(const State& s, VectorX<T>* dv) const;

  // Methods used to update cached quantities.
  void UpdateVelocitiesCache(const State& state, Cache* cache) const;
  void UpdateImpulsesCache(const State& state, Cache* cache) const;
  void UpdateCostCache(const State& state, Cache* cache) const;
  void UpdateMomentumCache(const State& state, Cache* cache) const;
  void UpdateSearchDirectionCache(const State& state, Cache* cache) const;
  void UpdateCostAndGradientsCache(const State& state, Cache* cache) const;

  // Quick access to mutable non thread-safe data.
  const SapSolverParameters& parameters() const {
    return non_thread_safe_data_.parameters;
  }
  const PreProcessedData& data() const { return non_thread_safe_data_.data; }
  SolverStats& mutable_stats() const { return non_thread_safe_data_.stats; }
  const SapConstraintsBundle<T>& constraints_bundle() const {
    return *data().constraints_bundle;
  }

    // All data stored by this class.
    mutable NonThreadSafeData non_thread_safe_data_;
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
