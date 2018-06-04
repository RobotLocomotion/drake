#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace implicit_stribeck {

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

struct Parameters {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

  Parameters() = default;

  /// The stiction tolerance vₛ for the slip velocity in the Stribeck
  /// function, in m/s. Roughly, for an externally applied tangential forcing
  /// fₜ and normal force fₙ, under "stiction", the slip velocity will be
  /// approximately to vₜ ≈ vₛ fₜ/(μfₙ). In other words, the maximum slip
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
  /// iteration. See ImplicitStribeckSolver::LimitDirectionChange() for
  /// details.
  double theta_max{0.25};  // about 15 degs
};

struct IterationStats {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IterationStats);

  IterationStats() = default;

  int num_iterations{0};
  double vt_residual{0.0};
  /// vt residual at each i-th iteration.
  std::vector<double> residuals;
  // At each iteration a velocity direction Δv is computed so that the
  // velocity at the next iteration is vᵏ⁺¹ = vᵏ + αΔv, where 0 < α < 1
  // (alpha) is a coefficient used to limit maximum angle change between
  // vₜᵏ⁺¹ and vₜᵏ, see ImplicitStribeckSolver::LimitDirectionChange().
  // This vector stores alpha for each iteration.
  std::vector<double> alphas{128};

  void Reset() {
    num_iterations = 0;
    vt_residual = -1.0;  // an invalid value.
    // Clear does not change a std::vector "capacity", and therefore there's
    // no reallocation (or deallocation) that could affect peformance.
    alphas.clear();
  }

  void Update(
      double iteration_residual, double iteration_alpha) {
    ++num_iterations;
    vt_residual = iteration_residual;

    residuals.push_back(iteration_residual);
    alphas.push_back(iteration_alpha);
  }
};

/// This class encapsulates the compliant contact model force computations as
/// described in detail in @ref drake_contacts.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// Note: The templated ScalarTypes are used in the KinematicsCache, but all
/// ImplicitStribeckSolvers use RigidBodyTree<double>.  This effectively implies
/// that we can e.g. AutoDiffXd with respect to the configurations, but not
/// the RigidBodyTree parameters.  The collision engine does not (yet) support
/// AutoDiffXd, so calls to that logic will throw errors at runtime.
template <typename T>
class ImplicitStribeckSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImplicitStribeckSolver)

  /// Instantiates a %ImplicitStribeckSolver.
  explicit ImplicitStribeckSolver(int nv);

  /// Sets data for the problem to be solved:
  ///   M⋅v = p* - δt⋅Dᵀ⋅fₜ(D⋅v)
  /// @warning This method stores constant references to the matrices and
  /// vectors passed as arguments. Therefore
  ///   1. they must outlived this class and,
  ///   2. changes to the problem data invalidate any solution performed by this
  ///      solver. In such a case, SetProblemData() and SolveWithGuess() must be
  ///      invoked again.
  void SetProblemData(
      EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> D,
      EigenPtr<const VectorX<T>> p_star,
      EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit ImplicitStribeckSolver(const ImplicitStribeckSolver<U>& other) :
      parameters_(other.parameters_) {}

  /// Solves for the friction forces using the implicit Stribeck approach
  /// described in this class's documentation.
  /// @returns the vector of generalized forces due to friction.
  /// @throws std::logic_error if v_guess is not of size `nv`, the number of
  /// generalized velocities specified at construction.
  ComputationInfo SolveWithGuess(double dt, const VectorX<T>& v_guess);

  const IterationStats& get_iteration_statistics() const {
    return statistics_;
  }

  const Parameters& get_solver_parameters() const {
    return parameters_;
  }

  void set_solver_parameters(const Parameters parameters) {
    parameters_ = parameters;
  }

  /// Get a constant reference to the vector of generalized friction forces.
  /// This is the solution to the problem.
  const VectorX<T>& get_generalized_forces() const {
    return fixed_size_workspace_.tau_f;
  }

  /// This method must be called after SolveWithGuess() to retrieve the vector
  /// of tangential velocities.
  Eigen::VectorBlock<const VectorX<T>> get_tangential_velocities() const {
    return variable_size_workspace_.vt();
  }

  const VectorX<T>& get_generalized_velocities() const {
    return fixed_size_workspace_.vk;
  }

 private:
  // For scalar-converting copy constructor.
  template <typename U>
  friend class ImplicitStribeckSolver;

  // Dimensionless modified Stribeck function defined as:
  // ms(x) = ⌈ mu * x * (2.0 - x),  x  < 1
  //         ⌊ mu                ,  x >= 1
  // where x corresponds to the dimensionless tangential speed
  // x = v / v_stribeck.
  static T ModifiedStribeck(const T& x, const T& mu);

  // Derivative of the dimensionless modified Stribeck function:
  // ms(x) = ⌈ mu * (2 * (1 - x)),  x  < 1
  //         ⌊ 0                 ,  x >= 1
  // where x corresponds to the dimensionless tangential speed
  // x = v / v_stribeck.
  static T ModifiedStribeckPrime(const T& speed_BcAc, const T& mu);

  // Returns alpha, s.t. v1 = v + alpha * dv.
  T LimitDirectionChange(const VectorX<T>& v, const VectorX<T>& dv) const;

  int nv_;  // Number of generalized velocities.
  int nc_;  // Number of contact points.

  // The parameters of the solver controlling the iteration strategy.
  Parameters parameters_;

  // Contains all the references that define the problem to be solved.
  // These references must remain valid at least from the time they are set with
  // SetProblemData() and until SolveWithGuess() returns.
  struct ProblemDataAliases {
    void Set(EigenPtr<const MatrixX<T>> M, EigenPtr<const MatrixX<T>> D,
             EigenPtr<const VectorX<T>> p_star,
             EigenPtr<const VectorX<T>> fn, EigenPtr<const VectorX<T>> mu) {
      M_ = M;
      D_ = D;
      p_star_ = p_star;
      fn_ = fn;
      mu_ = mu;
    }

    // The solver keeps references to the problem data but does not own it.
    EigenPtr<const MatrixX<T>> M_{nullptr};  // The mass matrix of the system.
    EigenPtr<const MatrixX<T>> D_{nullptr};  // The tangential velocities Jacobian.
    // The generalized momementum vector **before** friction is applied. For a
    // generalized velocity vector v, the generalized momentum vector is defined
    // as p = M ⋅ v.
    EigenPtr<const VectorX<T>> p_star_{nullptr};

    // At each contact point ic, fn_(ic) and mu_(ic) store the normal contact
    // force and friction coefficient, respectively. Both have size nc, the number
    // of contact points.
    EigenPtr<const VectorX<T>> fn_{nullptr};
    EigenPtr<const VectorX<T>> mu_{nullptr};
  };
  ProblemDataAliases problem_data_aliases_;

  // The solver's workspace allocated at construction time. Sizes only depend on
  // nv, the number of generalized velocities.
  // The size of the variables in this workspace MSUT remain fixed throghout the
  // lifetime of the solver. Do not resize any of them!.
  struct FixedSizeWorkspace {
    // Constructs a workspace with size only dependent on nv.
    explicit FixedSizeWorkspace(int nv) {
      vk.resize(nv);
      Rk.resize(nv);
      Delta_vk.resize(nv);
      Jk.resize(nv, nv);
      Jk_ldlt = std::make_unique<Eigen::LDLT<MatrixX<T>>>(nv);
      tau_f.resize(nv);
    }
    VectorX<T> vk;
    VectorX<T> Rk;
    MatrixX<T> Jk;
    VectorX<T> Delta_vk;
    VectorX<T> tau_f;  // Vector of generalized forces due to friction.
    // Factorization for the Newton-Raphson Jacobian.
    std::unique_ptr<Eigen::LDLT<MatrixX<T>>> Jk_ldlt;
  };
  mutable FixedSizeWorkspace fixed_size_workspace_;

  // The variables in this workspace can change size with each invocation of
  // SetProblemData() since the number of contact points nc can change.
  // The workspace only performs re-allocations if needed, meaning that previous
  // storeage is re-used if large engouh for the next problem data set.
  class VariableSizeWorkspace {
   public:
    explicit VariableSizeWorkspace(int initial_nc) {
      ResizeIfNeeded(initial_nc);
    }

    // Performs a resize of this workspace's variables only if the new size `nc`
    // is larger than the any of the other sizes with with this method was
    // called before.
    void ResizeIfNeeded(int nc) {
      nc_ = nc;
      if (capacity() >= nc) return;  // no-op if not needed.
      const int nf = 2 * nc;
      // Only reallocate if sizes from previous allocations are not sufficient.
      vtk.resize(nf);
      ftk.resize(nf);
      Delta_vtk.resize(nf);
      that.resize(nf);
      v_slip.resize(nc);
      mus.resize(nc);
      dmudv.resize(nc);
      // There is no reallocation if std::vector::capacity() >= nc.
      dft_dv.resize(nc);
    }

    // Returns the current (maximum) capacity of the workspace.
    int capacity() const {
      return vtk.size();
    }

    Eigen::VectorBlock<const VectorX<T>> vt() const {
      return vtk.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_vt() {
      return vtk.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_delta_vt() {
      return Delta_vtk.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_ft() {
      return ftk.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_that() {
      return that.segment(0, 2 * nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_v_slip() {
      return v_slip.segment(0, nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_mu() {
      return mus.segment(0, nc_);
    }

    Eigen::VectorBlock<VectorX<T>> mutable_dmudv() {
      return dmudv.segment(0, nc_);
    }

    std::vector<Matrix2<T>>& mutable_dft_dv() {
      return dft_dv;
    }

   private:
    // The number of contact points. This determines sizes in this workspace.
    int nc_;
    VectorX<T> Delta_vtk;
    VectorX<T> vtk;
    VectorX<T> ftk;
    VectorX<T> that;
    VectorX<T> v_slip;
    VectorX<T> mus; // Stribeck friction.
    VectorX<T> dmudv;
    std::vector<Matrix2<T>> dft_dv;
  };
  mutable VariableSizeWorkspace variable_size_workspace_;

  // We save solver statistics such as number of iterations and residuals so
  // that we can report them if requested.
  mutable IterationStats statistics_;
};

}  // namespace implicit_stribeck
}  // namespace multibody
}  // namespace drake
