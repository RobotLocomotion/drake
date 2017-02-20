#pragma once

#include <cmath>
#include <limits>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/src/LU/PartialPivLU.h>

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/// A first-order, half-explicit DAE solver. Such an approach solves DAEs of the
/// form:
/// <pre>
/// dx/dt = d(x(t), λ)
/// g(x) = 0
/// </pre>
/// where `d()` is an ordinary differential equation dependent upon constraint
/// forces `λ`, using the first order relationship:
/// <pre>
/// x(t+Δt) = x(t) + Δt⋅d(x(t), λ)
/// </pre>
/// to solve the following nonlinear system of equations for λ:
/// <pre>
/// g(x(t)+ Δt⋅d(x(t), λ) = 0
/// </pre>
/// After the value of `λ` has been obtained *implicitly*, `x(t+Δt)` is computed
/// *explicitly* using the second to last equation above.

/// Let us now consider the state variables as representing a mechanical system
/// with generalized coordinates `q` and generalized velocities `v`. We also
/// define the "quasi-coordinates" `ꝗ` (pronounced "qbar"), where `dꝗ/dt` is
/// equivalent to the generalized velocities. We can then designate the matrix
/// of partial derivatives of the constraint equations
/// taken with respect to the quasi-coordinates as `J`. Formally:
/// <pre>
/// J = ∂g/∂q
/// </pre>
/// The time derivative of the constraint equations `dg/dt = Jv` yields-
/// from the dual relationship between velocities and forces- generalized forces
/// (`f`) via the relationship `f = Jᵀλ`. We now reformulate the equations
/// above as:
/// <pre>
/// v(t+Δt) = v(t) + Δt⋅e() + M⁻¹Jᵀλ
/// q(t+Δt) = q(t) + Δt⋅Nv(t+Δt)
/// g(q(t+Δt), v(t+Δt)) = 0
/// </pre>
/// where `e(q(t), v(t))` gives the time derivatives of the generalized velocity
/// variables- now independently of `λ`- and `M` is the generalized inertia
/// matrix of the mechanical system. `J` and `N` are both dependent upon
/// `q(t)`. Determining the constraint forces via Newton-Raphson (the standard
/// algorithm for solving nonlinear systems of equations)
/// requires computing the Jacobian matrix `∂g/∂λ`. Focusing
/// only on configuration-dependent constraints, i.e., assuming that `g(.)` is a
/// function only of `q(t+Δt)`, we derive:
/// <pre>
/// ∂g(q(t+Δt))/∂λ = ∂g/∂q(t+Δt)⋅∂q(t+Δt)/∂λ
/// </pre>
/// where ∂q(t+Δt)/∂λ is straightforward to obtain in closed form:
/// <pre>
/// ∂q(t+Δt)/∂λ = Δt⋅NM⁻¹Jᵀ
/// </pre>
/// but `∂g(q(t+Δt))/∂λ` is not. This exercise shows that the necessary
/// Jacobian matrix is not neatly provided as the output of some typical
/// piece of code: our particular implementation uses finite
/// differencing to compute `∂g/∂λ`, as a result.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
template <class T>
class HalfExplicitDAE1Solver : public IntegratorBase<T> {
 public:
  ~HalfExplicitDAE1Solver() override = default;

  // Disable copy, assign, and move.
  HalfExplicitDAE1Solver(const HalfExplicitDAE1Solver<T>& other) = delete;
  HalfExplicitDAE1Solver& operator=(const HalfExplicitDAE1Solver<T>& other) =
      delete;

  /**
   * Constructs a fixed-step integrator for a given system using the given
   * context for initial conditions.
   * @param system A reference to the system to be simulated
   * @param max_step_size The maximum (fixed) step size; the integrator will
   *                      not take larger step sizes than this.
   * @param context Pointer to the context (nullptr is ok, but the caller
   *                must set a non-null context before Initialize()-ing the
   *                integrator).
   * @sa Initialize()
   */
  HalfExplicitDAE1Solver(const System<T>& system, const T& max_step_size,
                          Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
    derivs_ = system.AllocateTimeDerivatives();
  }

  /**
   * Solver does not currently support error estimation.
   */
  bool supports_error_estimation() const override { return false; }

  /// Solver does not provide an error estimate.
  int get_error_estimate_order() const override { return 0; }

  /// Gets the tolerance for the norm of the error in the algebraic constraint
  /// equation solutions.
  double get_constraint_error_tolerance() const { return constraint_tol_; }

  /// Sets the tolerance for the norm of the error in the algebraic constraint
  /// equation solutions.
  void set_constraint_error_tolerance(double tol) {
    if (tol < 0)
      throw std::logic_error("Tolerance must be non-negative.");
    constraint_tol_ = tol;
  }

 private:
  // Structure for holding return values from the line search procedure used
  // in the context of Newton-Raphson.
  struct LineSearchOutput {
    // New lambda iterate.
    Eigen::VectorXd lambda_new;

    // Value of the algebraic constraint functions: g(lambda).
    Eigen::VectorXd goutput;

    // Objective function value 1/2*dot(g(lambda), g(lambda))
    double fnew{std::numeric_limits<double>::max()};
  };

  LineSearchOutput search_line(const Eigen::VectorXd& lambda,
                               const Eigen::VectorXd& dlambda,
                               double fold,
                               const Eigen::VectorXd& gradient,
                               const Eigen::MatrixXd& J,
                               double dt) const;
  void DoStepOnceFixedSize(const T& dt) override;
  Eigen::MatrixXd CalcConstraintSpaceInertiaMatrix(
      const systems::Context<T>& context, const Eigen::MatrixXd& J) const;
  void UpdateContinuousState(Context<T>* context,
                             const Eigen::VectorXd& lambda,
                             const Eigen::MatrixXd& J,
                             double dt) const;
  Eigen::MatrixXd CalcConstraintJacobian(
      const systems::Context<T>& context) const;

  // Calculates the m x ℓ Jacobian matrix of the partial derivatives of the m
  // algebraic constraint equations computed with respect to the ℓ constraint
  // variables.
  Eigen::MatrixXd CalcAlgebraicJacobian(const Context<T>& context,
                                        const Eigen::VectorXd& lambda,
                                        const Eigen::MatrixXd& Jc,
                                        double dt) const;

  // The maximum number of Newton-Raphson iterations.
  int max_nr_iterations_{std::numeric_limits<int>::max()};

  // The tolerance for the norm of the error in the algebraic constraint
  // equation solutions.
  double constraint_tol_{std::sqrt(std::numeric_limits<double>::epsilon())};

  // Temporary copies of generalized coordinates and velocities.
  Eigen::VectorXd qsave_, vsave_;

  // These are pre-allocated temporaries for use by integration
  std::unique_ptr<ContinuousState<T>> derivs_;
  std::unique_ptr<BasicVector<T>> qdot_;

  // Matrix factorizations, for which the memory can be used repeatedly.
  Eigen::LLT<Eigen::MatrixXd> chol_;
  Eigen::FullPivLU<Eigen::MatrixXd> LU_;
};

}  // namespace systems
}  // namespace drake

