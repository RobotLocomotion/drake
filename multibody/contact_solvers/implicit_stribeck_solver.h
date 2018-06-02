#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

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

    // The number of linear iterations performed by the linear solver at each
    // Newton-Raphsone iteration.
    std::vector<int> linear_iterations{128};

    // The residual of the linear solver at each Newton-Raphson iteration.
    std::vector<double> linear_residuals{128};

    void Reset() {
      num_iterations = 0;
      vt_residual = -1.0;  // an invalid value.
      // Clear does not change a std::vector "capacity", and therefore there's
      // no reallocation (or deallocation) that could affect peformance.
      alphas.clear();
      linear_iterations.clear();
      linear_residuals.clear();
    }

    void Update(
        double iteration_residual, double iteration_alpha,
        int lin_iterations, double lin_residuals) {
      ++num_iterations;
      vt_residual = iteration_residual;

      residuals.push_back(iteration_residual);
      alphas.push_back(iteration_alpha);
      linear_iterations.push_back(lin_iterations);
      linear_residuals.push_back(lin_residuals);
    }
  };

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
  VectorX<T> SolveWithGuess(double dt, const VectorX<T>& v_guess);

  const IterationStats& get_iteration_statistics() const {
    return statistics_;
  }

  const Parameters& get_solver_parameters() const {
    return parameters_;
  }

  void set_solver_parameters(const Parameters parameters) {
    parameters_ = parameters;
  }

  /// This method must be called after SolveWithGuess() to retrieve the vector
  /// of tangential velocities.
  const VectorX<T>& get_tangential_velocities() const {
    return vtk;
  }

  const VectorX<T>& get_generalized_velocities() const {
    return vk;
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

  void ResizeSolverWorkspaceAsNeeded(int nc);

  int nv_;  // Number of generalized velocities.
  int nc_;  // Number of contact points.

  // The parameters of the solver controlling the iteration strategy.
  Parameters parameters_;

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

  // The solver's workspace allocated at construction time:
  VectorX<T> vk;
  VectorX<T> Rk;
  MatrixX<T> Jk;
  VectorX<T> Delta_vk;

  // Workspace that needs to be allocated everytime nc changes.
  VectorX<T> Delta_vtk;
  VectorX<T> vtk;
  VectorX<T> ftk;
  VectorX<T> that;
  VectorX<T> v_slip;
  VectorX<T> mus; // Stribeck friction.
  VectorX<T> dmudv;
  std::vector<Matrix2<T>> dft_dv;

  // We save solver statistics such as number of iterations and residuals so
  // that we can report them if requested.
  IterationStats statistics_;
};

}  // namespace multibody
}  // namespace drake
