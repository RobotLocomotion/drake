#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/parameters.h"

namespace drake {
namespace systems {

/// A scalar initial value problem (or IVP) representation class, that allows
/// for the evaluation of the x(t; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) solution function to the
/// given ODE dx/dt = f(t, x; 𝐤₀, 𝐤₁, ..., 𝐤ₙ), where f : t ⨯ x →  ℝ , t ∈ ℝ ,
/// x ∈ ℝ ,  𝐤₀ ∈ ℝ ᵐ⁰, 𝐤₁ ∈ ℝ ᵐ¹, ..., 𝐤ₙ ∈ ℝ ᵐᶻ, provided an initial
/// condition x(t₀; 𝐩) = x₀.
///
/// For further insight into its use, consider the following examples:
///
/// - The population growth of an hypothetical bacteria colony is described
///   by dN/dt = r * N. The colony has N₀ subjects at time t₀. In this
///   context, x ≜ N, x₀ ≜ N₀, 𝐤₀ ≜ [r], dx/dt = f(t, x; 𝐤₀) = 𝐤₀₁ * x.
///
/// - The charge Q stored in the capacitor of a (potentially equivalent) series
///   RC circuit driven by a time varying voltage source E(t) can be described
///   by dQ/dt = (E(t) - Q / Cs) / Rs, where Rs refers to the resistor's
///   resistance and Cs refers to the capacitor's capacitance. In this context,
///   and assuming an initial stored charge Q₀ at time t₀, x ≜ Q, 𝐤₀ ≜ [Rs, Cs],
///   x₀ ≜ Q₀, dx/dt = f(t, x; 𝐤₀) = (E(t) - x / 𝐤₀₂) / 𝐤₀₁.
///
/// @tparam T The ℝ domain scalar type, which must be a valid Eigen scalar.
///
/// @note
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
template <typename T>
class ScalarInitialValueProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarInitialValueProblem);

  /// Scalar ODE dx/dt = f(t, x; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) function type.
  ///
  /// @param t The independent variable t ∈ ℝ .
  /// @param x The dependent variable x ∈ ℝ .
  /// @param k The parameter sequence 𝐤₀ ∈ ℝ ᵐ⁰, 𝐤₁ ∈ ℝ ᵐ¹, ..., 𝐤ₙ ∈ ℝ ᵐᶻ.
  /// @return The derivative dx/dt ∈ ℝ .
  typedef std::function<T(const T& t, const T& x,
                          const Parameters<T>& k)> ScalarODEFunction;

  /// Constructs a scalar IVP described by the given @p scalar_ode_function,
  /// using given @p default_initial_time and @p default_initial_state as
  /// initial conditions, and parameterized with @p default_parameters by
  /// default.
  ///
  /// @param scalar_ode_function The scalar ODE function f(t, x; 𝐩) that
  /// describes the state evolution over time.
  /// @param default_initial_time The default initial time t₀.
  /// @param default_initial_state The default initial state x₀.
  /// @param default_parameters The default parameter sequence 𝐤₀, 𝐤₁, ..., 𝐤ₙ.
  ScalarInitialValueProblem(const ScalarODEFunction& scalar_ode_function,
                            const T& default_initial_time,
                            const T& default_initial_state,
                            const Parameters<T>& default_parameters);

  /// Solves the IVP for the given time @p t, using the default parameter
  /// sequence and default initial conditions (t₀, x₀).
  ///
  /// @param t The time t to solve the IVP up to.
  /// @return The IVP solution x(@p t ; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) for
  /// x(t₀; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) = x₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  /// given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T Solve(const T& t) const {
    return this->generalized_ivp_->Solve(t).GetAtIndex(0);
  }

  /// Solves the IVP for the given time @p t, starting at the given initial time
  /// @p t0, using the default parameter sequence 𝐤₀, 𝐤₁, ..., 𝐤ₙ and default
  /// initial state x₀.
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP up to.
  /// @return The IVP solution x(@p t ; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) for
  /// x(@p t0 ; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) = x₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  /// @p t0 .
  /// @warning This method will abort if preconditions are not met.
  inline T Solve(const T& t0, const T& t) const {
    return this->generalized_ivp_->Solve(t0, t).GetAtIndex(0);
  }

  /// Solves the IVP for the given time @p t using the given parameter sequence
  /// @p k and default initial conditions (t₀, x₀) .
  ///
  /// @param t The time to solve the IVP up to.
  /// @param k The parameter sequence for the IVP.
  /// @return The IVP solution x(@p t ; @p k ) for x(t₀; @p k ) = x₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  /// given on construction.
  /// @pre The length and order of the given parameter sequence @p k, and
  /// the dimension of each of its elements must match that the default
  /// parameter sequence 𝐤₀, 𝐤₁, ..., 𝐤ₙ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T Solve(const T& t, const Parameters<T>& k) const {
    return this->generalized_ivp_->Solve(t, k).GetAtIndex(0);
  }

  /// Solves the IVP for the given time @p t using the given parameter
  /// sequence @p k and starting at time @p t0 with default initial state x₀.
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP up to.
  /// @param k The parameter sequence for the IVP.
  /// @return The IVP solution x(@p t ; @p k ) for x(@p t0 ; @p k ) = x₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  /// @p t0 .
  /// @pre The length and order of the given parameter sequence @p k, and
  /// the dimension of each of its elements must match that the default
  /// parameter sequence 𝐤₀, 𝐤₁, ..., 𝐤ₙ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T Solve(const T& t0, const T& t, const Parameters<T>& k) const {
    return this->generalized_ivp_->Solve(t0, t, k).GetAtIndex(0);
  }

  /// Solves the IVP for the given time @p t using the given parameter
  /// sequence @p k and given initial conditions (@p t0 , @p x0 ).
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP up to.
  /// @param x0 The initial state of the IVP.
  /// @param k The parameter sequence for the IVP.
  /// @return The IVP solution x(@p t ; @p k ) for x(@p t0 ; @p k ) = @p x0 .
  /// @pre The time @p t must be larger than or equal to the initial time
  /// @p t0 .
  /// @pre The dimension of the given initial state @p x0 must match that of the
  /// default initial state x₀ given on construction.
  /// @pre The length and order of the given parameter sequence @p k, and
  /// the dimension of each of its elements must match that the default
  /// parameter sequence 𝐤₀, 𝐤₁, ..., 𝐤ₙ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T Solve(const T& t0, const T& x0,
                 const T& t, const Parameters<T>& k) const {
    const BasicVector<T> initial_state(
        VectorX<T>::Constant(1, x0));
    return this->generalized_ivp_->Solve(
        t0, initial_state, t, k).GetAtIndex(0);
  }

  /// Resets the internal integrator instance.
  /// @return The new integrator instance.
  /// @tparam I The integrator type, which must be an IntegratorBase subclass.
  template <typename I>
  inline IntegratorBase<T>* reset_integrator() {
    return generalized_ivp_->template reset_integrator<I>();
  }

  inline const IntegratorBase<T>* get_integrator() const {
    return generalized_ivp_->get_integrator();
  }

  inline IntegratorBase<T>* get_mutable_integrator() {
    return generalized_ivp_->get_mutable_integrator();
  }

 private:
  // Numerical integrator used for function evaluation.
  std::unique_ptr<InitialValueProblem<T>> generalized_ivp_;
};

}  // namespace systems
}  // namespace drake
