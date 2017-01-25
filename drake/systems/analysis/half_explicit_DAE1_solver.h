#pragma once

#include <Eigen/Dense>
#include <eigen3/Eigen/src/LU/PartialPivLU.h>
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

/// A first-order, half-explicit DAE solver. This approach solves DAEs of the
/// form:
/// <pre>
/// dx/dt = f(x(t), λ)
/// g(x) = 0
/// </pre>
/// using the first order relationship:
/// <pre>
/// x(t+Δt) = x(t) + Δt⋅f(x(t), λ)
/// </pre>
/// to solve the following nonlinear system of equations for λ:
/// <pre>
/// g(x(t)+Δt⋅f(x(t), λ) = 0
/// </pre>
/// After this value of λ has been obtained *implicitly*, x(t+Δt) is computed
/// *explicitly* using the second to last equation above.
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
                               double dt) const;
  void DoStepOnceFixedSize(const T& dt) override;
  Eigen::MatrixXd CalcConstraintSpaceInertiaMatrix(
      const systems::Context<T>& context, const Eigen::MatrixXd& J) const;
  void UpdateContinuousState(Context<T>* context,
                             const Eigen::VectorXd& lambda,
                             double dt) const;
  Eigen::MatrixXd CalcConstraintJacobian(
      const systems::Context<T>& context) const;

  // Calculates the m x ℓ Jacobian matrix of the partial derivatives of the m
  // algebraic constraint equations computed with respect to the ℓ constraint
  // variables.
  Eigen::MatrixXd CalcAlgebraicJacobian(const Context<T>& context,
                                        const Eigen::VectorXd& lambda,
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

