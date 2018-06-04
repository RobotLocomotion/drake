#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {

/// The result from ImplicitStribeckSolver::SolveWithGuess() used to report the
/// success or failure of the solver.
enum ComputationInfo {
  /// Successful computation.
  Success = 0,

  /// The maximum number of iterations was reached.
  MaxIterationsReached = 1,

  /// The linear solver used within the Newton-Raphson loop failed.
  /// This might be caused by a divergent iteration that led to an invalid
  /// Jacobian matrix.
  LinearSolverFailed = 2
};

/// These are the paramters controlling the iteration process of the
/// ImplicitStribeckSolver solver.
struct Parameters {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

  Parameters() = default;

  /// The stiction tolerance vₛ for the slip velocity in the Stribeck
  /// function, in m/s. Roughly, for an externally applied tangential forcing
  /// fₜ and normal force fₙ, under "stiction", the slip velocity will be
  /// approximately vₜ ≈ vₛ fₜ/(μfₙ). In other words, the maximum slip
  /// error of the Stribeck approximation occurs at the edge of the friction
  /// cone when fₜ = μfₙ and vₜ = vₛ.
  double stiction_tolerance{1.0e-4};  // 0.1 mm/s

  /// The maximum number of iterations allowed for the Newton-Raphson
  /// iterative solver.
  int max_iterations{100};

  /// The tolerance to monitor the convergence of the tangential velocities.
  /// This number specifies a tolerance relative to the value of the
  /// stiction_tolerance and thus it is dimensionless. Using a tolerance
  /// relative to the value of the stiction_tolerance is necessary in order
  /// to capture transitions to stiction that would require an accuracy in the
  /// value of the tangential velocities smaller than that of the
  /// "Stribeck stiction region" (the circle around the origin with radius
  /// stiction_tolerance).
  /// Typical value is about 1%.
  double tolerance{1.0e-2};

  /// The maximum angle change in tangential velocity allowed at each
  /// iteration. See the class's documentation for ImplicitStribeckSolver.
  double theta_max{0.25};  // about 15 degs
};

/// Struct used to store information about the iteration process performed by
/// ImplicitStribeckSolver.
struct IterationStats {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IterationStats);

  IterationStats() = default;

  /// The number of iterations performed by the last ImplicitStribeckSolver
  /// solve.
  int num_iterations{0};

  /// The residual in the tangential velocities, in m/s. Upon convergence of
  /// the solver this value should be smaller than Parameters::tolerance times
  /// Parameters::stiction_tolerance.
  double vt_residual{0.0};

  /// (Advanced) Residual in the tengential velocities at each k-th iteration.
  /// After ImplicitStribeckSolver solved a problem, this vector will have size
  /// num_iterations.
  std::vector<double> residuals;

  /// (Advanced) At each iteration the generalized velocities update is limited
  /// as: vᵏ⁺¹ = vᵏ + αΔv
  /// where α is computed to limit the maximum angle change of the tangential
  /// velocities. See [Uchida et al., 2015] for details.
  /// This vector stores α for each k-th iteration of the solver and will
  /// therefore be of size num_iterations.
  std::vector<double> alphas{128};

  /// (Internal) Used by ImplicitStribeckSolver to reset statistics.
  void Reset() {
    num_iterations = 0;
    vt_residual = -1.0;  // an invalid value.
    // Clear does not change a std::vector "capacity", and therefore there's
    // no reallocation (or deallocation) that could affect peformance.
    alphas.clear();
  }

  /// (Internal) Used by ImplicitStribeckSolver to update statistics.
  void Update(
      double iteration_residual, double iteration_alpha) {
    ++num_iterations;
    vt_residual = iteration_residual;

    residuals.push_back(iteration_residual);
    alphas.push_back(iteration_alpha);
  }
};

/// This solver is used to solve mechanical systems with contact using a
/// modified Stribeck model of friction: <pre>
///             q̇ = N(q) v
///   (1)  M(q)⋅v̇ = τ + Jₙᵀ(q)⋅fₙ(q, v) + Jₜᵀ(q)⋅fₜ(v)
/// </pre>
/// where `v ∈ ℝⁿᵛ` is the vector of generalized velocities, `M(q) ∈ ℝⁿᵛˣⁿᵛ` is
/// the mass matrix, `Jₙᵀ(q) ∈ ℝⁿᶜˣⁿᵛ` is the Jacobian of normal separation
/// velocities, `Jₜᵀ(q) ∈ ℝ²ⁿᶜˣⁿᵛ` is the Jacobian of tangent velocities,
/// `fₙ ∈ ℝⁿᶜ` is the vector of normal contact forces, `fₜ ∈ ℝ²ⁿᶜ` is the vector
/// of tangent friction forces and τ ∈ ℝⁿᵛ is a vector of generalized forces
/// containing externally applied forces as well as Coriolis and gyroscopic
/// terms. Since %ImplicitStribeckSolver uses a Stribeck model for friction,
/// `fₜ(v)` implicitly depends on the vector of generalized velocities.
///
/// Equations (1) is discretized in time using a first order semi-implicit Euler
/// scheme with time step `dt` as: <pre>
///              qⁿ⁺¹ = qⁿ + δt N(qⁿ)⋅vⁿ⁺¹
///   (2)  M(qⁿ)⋅vⁿ⁺¹ =
///            M(qⁿ)⋅vⁿ + δt (τⁿ + Jₙᵀ(qⁿ)⋅fₙ(qⁿ, vⁿ) + Jₜᵀ(qⁿ)⋅fₜ(vⁿ⁺¹))
/// </pre>
/// The equation for the generalized velocities in Eq. (2) is rewritten as:
/// <pre>
///   (3)  M⋅vⁿ⁺¹ = p* + δt Jₜᵀ⋅fₜ(vⁿ⁺¹))
/// </pre>
/// where `p* = M⋅vⁿ + δt (τⁿ + Jₙᵀ⋅fₙ` is the generalized momentum that the
/// system would have in the abscence of friction forces and, for simplicity, we
/// have only kept the explicit functional dependencies in generalized
/// velocities. Notice that %ImplicitStribeckSolver uses a precomputed value of
/// the normal forces. These normal forces could be available for instance if
/// using a compliant contact apporach, for which normal forces are a funciton
/// of the state.
///
/// %ImplicitStribeckSolver is designed to solve, implicitly, the system in
/// Eq. (3) for the next time step vector of generalized velocities `vⁿ⁺¹`.
/// The solver uses a Newton-Raphson iteration to compute an update `Δvᵏ` at the
/// k-th Newton-Raphson iteration. Once `Δvᵏ` is computed, the solver limits the
/// change in the tangential velocities `Δvₜᵏ = Jₜᵀ⋅Δvᵏ` using the approach
/// described in [Uchida et al., 2015]. This approach limits the maximum angle
/// change θ between two successive iterations in the tangential velocity. The
/// maximum angle change between iterations is given by the parameter
/// Parameters::theta_max.
///
/// Uchida, T.K., Sherman, M.A. and Delp, S.L., 2015.
///   Making a meaningful impact: modelling simultaneous frictional collisions
///   in spatial multibody systems. Proc. R. Soc. A, 471(2177), p.20140859.
///
/// @tparam T The type of mathematical object being added.
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class ImplicitStribeckSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitStribeckSolver)

  /// Instantiates a solver for a problem with `nv` generalized velocities.
  /// @throws std::exception if nv is non-positive.
  explicit ImplicitStribeckSolver(int nv);

  /// Sets data for the problem to be solved as outlined by Eq. (3) in this
  /// class's documentation: <pre>
  ///   M⋅v = p* + δt Jₜᵀ⋅fₜ(v)
  /// </pre>
  /// Refere to this class's documentation for further details on the structure
  /// of the problem and the solution strategy.
  /// In the documented parameters below, `nv` is the number of generalized
  /// velocities and `nc` is the number of contact points.
  ///
  /// @param[in] M
  ///   The mass matrix of the system, of size `nv x nv`.
  /// @param[in] Jt
  ///   The tangential velocities Jacobian, of sice `2nc x nv`.
  /// @param[in] p_star
  ///   The generalized momentum the system would have at `n + 1` if friction
  ///   forces where zero.
  /// @param[in] fn
  ///   A vector of size `nc` containing the normal force at each contact point.
  /// @param[in] mu
  ///   A vector of size `nc` containing the friction coefficient at each
  ///   contact point. The solver makes no distinction between static and
  ///   dynamic coefficients of friction or, similarly, the solver assumes the
  ///   static and dynamic coefficients of friction are the same.
  ///
  /// @warning This method stores constant references to the matrices and
  /// vectors passed as arguments. Therefore
  ///   1. they must outlive this class and,
  ///   2. changes to the problem data invalidate any solution performed by this
  ///      solver. In such a case, SetProblemData() and SolveWithGuess() must be
  ///      invoked again.
  void SetProblemData(
      EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> Jt,
      EigenPtr<const VectorX<T>> p_star,
      EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu);

  /// Solves for the generalized velocities satisfying Eq. (3) in this class's
  /// documentation.
  /// @returns ComputationInfo::Success if the iteration converges. All other
  /// values of ComputationInfo report different failure modes.
  /// Uses `this` solver accessors to retrieve the last computed solution.
  ///
  /// @param[in] dt The time step used advance the solution in time.
  /// @param[in] v_guess The initial guess used in by the Newton-Raphson
  /// iteration.
  ///
  /// @throws std::logic_error if `v_guess` is not of size `nv`, the number of
  /// generalized velocities specified at construction.
  ComputationInfo SolveWithGuess(double dt, const VectorX<T>& v_guess);

  /// Returns statistics recorded during the last call to SolveWithGuess().
  /// See IterationStats for details.
  const IterationStats& get_iteration_statistics() const {
    return statistics_;
  }

  /// Returns the current set of parameters controlling the iteration process.
  /// See Parameters for details.
  const Parameters& get_solver_parameters() const {
    return parameters_;
  }

  /// Sets the parameters to be used by the solver.
  /// See Parameters for details.
  void set_solver_parameters(const Parameters parameters) {
    parameters_ = parameters;
  }

  /// Returns a constant reference to the last solved vector of generalized
  /// friction forces.
  const VectorX<T>& get_generalized_forces() const {
    return fixed_size_workspace_.tau_f;
  }

  /// Returns a constant reference to the last solved vector of tangential
  /// forces. This method returns an `Eigen::VectorBlock` referencing a vector
  /// of size `nc` in accordance to the data last set with SetProblemData().
  Eigen::VectorBlock<const VectorX<T>> get_tangential_velocities() const {
    return variable_size_workspace_.vt();
  }

  /// Returns a constant reference to the last solved vector of generalized
  /// velocities.
  const VectorX<T>& get_generalized_velocities() const {
    return fixed_size_workspace_.v;
  }

 private:
  // Contains all the references that define the problem to be solved.
  // These references must remain valid at least from the time they are set with
  // SetProblemData() and until SolveWithGuess() returns.
  struct ProblemDataAliases {
    // Sets the references to the data defining the problem.
    void Set(EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> D,
             EigenPtr<const VectorX<T>> p_star,
             EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu) {
      M_ptr = M;
      D_ptr = D;
      p_star_ptr = p_star;
      fn_ptr = fn;
      mu_ptr = mu;
    }

    // The mass matrix of the system.
    EigenPtr<const MatrixX<T>> M_ptr{nullptr};
    // The tangential velocities Jacobian.
    EigenPtr<const MatrixX<T>> D_ptr{nullptr};
    // The generalized momementum vector **before** friction is applied.
    EigenPtr<const VectorX<T>> p_star_ptr{nullptr};
    // At each contact point ic, fn(ic) and mu(ic) store the normal contact
    // force and friction coefficient, respectively. Both have size nc, the
    // number of contact points.
    EigenPtr<const VectorX<T>> fn_ptr{nullptr};
    EigenPtr<const VectorX<T>> mu_ptr{nullptr};
  };

  // The solver's workspace allocated at construction time. Sizes only depend on
  // nv, the number of generalized velocities.
  // The size of the variables in this workspace MUST remain fixed throghout the
  // lifetime of the solver. Do not resize any of them!.
  struct FixedSizeWorkspace {
    // Constructs a workspace with size only dependent on nv.
    explicit FixedSizeWorkspace(int nv) {
      v.resize(nv);
      R.resize(nv);
      Delta_v.resize(nv);
      J.resize(nv, nv);
      J_ldlt = std::make_unique<Eigen::LDLT<MatrixX<T>>>(nv);
      tau_f.resize(nv);
    }
    // Vector of generalized velocities.
    VectorX<T> v;
    // Newton-Raphson residual.
    VectorX<T> R;
    // Newton-Raphson Jacobian, i.e. Jᵢⱼ = ∂Rᵢ/∂vⱼ.
    MatrixX<T> J;
    // Solution to Newton-Raphson update, i.e. Δv = -J⁻¹⋅R.
    VectorX<T> Delta_v;
    // Vector of generalized forces due to friction.
    VectorX<T> tau_f;
    // LDLT Factorization of the Newton-Raphson Jacobian J.
    std::unique_ptr<Eigen::LDLT<MatrixX<T>>> J_ldlt;
  };

  // The variables in this workspace can change size with each invocation of
  // SetProblemData() since the number of contact points nc can change.
  // The workspace only performs re-allocations if needed, meaning that previous
  // storage is re-used if large enough for the next problem data set.
  // This class provides accessors that return Eigen blocks of size consistent
  // with the data currently stored, even if the (maximum) capacity is larger
  // than the data size.
  class VariableSizeWorkspace {
   public:
    explicit VariableSizeWorkspace(int initial_nc) {
      ResizeIfNeeded(initial_nc);
    }

    // Performs a resize of this workspace's variables only if the new size `nc`
    // is larger than capcity() in order to reuse previously allocated space.
    void ResizeIfNeeded(int nc) {
      nc_ = nc;
      if (capacity() >= nc) return;  // no-op if not needed.
      const int nf = 2 * nc;
      // Only reallocate if sizes from previous allocations are not sufficient.
      vt_.resize(nf);
      ft_.resize(nf);
      Delta_vt_.resize(nf);
      that_.resize(nf);
      v_slip_.resize(nc);
      mus_.resize(nc);
      dft_dv_.resize(nc);
    }

    // Returns the current (maximum) capacity of the workspace.
    int capacity() const {
      return vt_.size();
    }

    Eigen::VectorBlock<const VectorX<T>> vt() const {
      return vt_.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_vt() {
      return vt_.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_delta_vt() {
      return Delta_vt_.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_ft() {
      return ft_.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_that() {
      return that_.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_v_slip() {
      return v_slip_.segment(0, nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_mu() {
      return mus_.segment(0, nc_);
    }

    std::vector<Matrix2<T>>& mutable_dft_dv() {
      return dft_dv_;
    }

   private:
    // The number of contact points. This determines sizes in this workspace.
    int nc_;
    VectorX<T> Delta_vt_;  // Δvₜᵏ = Jₜ⋅Δvᵏ, in ℝ²ⁿᶜ, for the k-th NR iteration.
    VectorX<T> vt_;        // vₜᵏ, in ℝ²ⁿᶜ.
    VectorX<T> ft_;        // fₜᵏ, in ℝ²ⁿᶜ.
    VectorX<T> that_;      // Tangential directions, t̂ᵏ. In ℝ²ⁿᶜ.
    VectorX<T> v_slip_;    // vₛᵏ = ‖vₜᵏ‖, in ℝⁿᶜ.
    VectorX<T> mus_;       // (modified) Stribeck friction, in ℝⁿᶜ.
    // Vector of size nc storing ∂fₜ/∂vₜ (in ℝ²ˣ²) for each contact point.
    std::vector<Matrix2<T>> dft_dv_;
  };

  // Dimensionless modified Stribeck function defined as:
  // ms(x) = ⌈ mu * x * (2.0 - x),  x  < 1
  //         ⌊ mu                ,  x >= 1
  // where x corresponds to the dimensionless tangential speed
  // x = v / v_stribeck.
  // The solver uses this modified Stribeck function for thwo reasons:
  //   1. Static and dynamic friction coefficients are the same. This avoid
  //      regions of negative slope. If the slope is always positive the
  //      implicit update is unconditionally stable.
  //   2. Non-zero derivative at x = 0 (zero slip velocity). This provides a
  //      good strong gradient in the neighborhood to zero slip velocities that
  //      aids in finding a good solution update.
  static T ModifiedStribeck(const T& x, const T& mu);

  // Derivative of the dimensionless modified Stribeck function:
  // ms(x) = ⌈ mu * (2 * (1 - x)),  x  < 1
  //         ⌊ 0                 ,  x >= 1
  // where x corresponds to the dimensionless tangential speed
  // x = v / v_stribeck.
  static T ModifiedStribeckPrime(const T& speed_BcAc, const T& mu);

  // Given a 2D vector of tangent velocities v and its update dv, this method
  // computes a coefficient α so that the tangential velocity in the next update
  //   vᵏ⁺¹ = vᵏ + α Δv
  // forms an angle θ with vᵏ that is limited to have a maximum value given by
  // Parameters::theta_max.
  // Please refer to [Uchida et al., 2015] for further details.
  T LimitDirectionChange(const VectorX<T>& v, const VectorX<T>& dv) const;

  int nv_;  // Number of generalized velocities.
  int nc_;  // Number of contact points.
  // The parameters of the solver controlling the iteration strategy.
  Parameters parameters_;
  ProblemDataAliases problem_data_aliases_;
  mutable FixedSizeWorkspace fixed_size_workspace_;
  mutable VariableSizeWorkspace variable_size_workspace_;

  // We save solver statistics such as number of iterations and residuals so
  // that we can report them if requested.
  mutable IterationStats statistics_;
};

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake
