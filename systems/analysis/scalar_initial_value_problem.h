#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/initial_value_problem.h"

namespace drake {
namespace systems {

/// A thin wrapper of the InitialValueProblem class to provide a simple
/// interface when solving scalar initial value problems i.e. when evaluating
/// the x(t; 𝐤) solution function to the given ODE dx/dt = f(t, x; 𝐤),
/// where f : t ⨯ x →  ℝ , t ∈ ℝ, x ∈ ℝ, 𝐤 ∈ ℝᵐ, along with an initial
/// condition x(t₀; 𝐤) = x₀. The parameter vector 𝐤 allows for generic IVP
/// definitions, which can later be solved for any instance of said vector.
/// Note the distinction from general initial value problems where
/// f : t ⨯ x → ℝⁿ and x ∈ ℝⁿ, addressed by the class being wrapped.
///
/// For further insight into its use, consider the following examples:
///
/// - The population growth of an hypothetical bacteria colony is described
///   by dN/dt = r * N. The colony has N₀ subjects at time t₀. In this
///   context, x ≜ N, x₀ ≜ N₀, 𝐤 ≜ [r], dx/dt = f(t, x; 𝐤) = 𝐤₁ * x.
///
/// - The charge Q stored in the capacitor of a (potentially equivalent) series
///   RC circuit driven by a time varying voltage source E(t) can be described
///   by dQ/dt = (E(t) - Q / Cs) / Rs, where Rs refers to the resistor's
///   resistance and Cs refers to the capacitor's capacitance. In this context,
///   and assuming an initial stored charge Q₀ at time t₀, x ≜ Q, 𝐤 ≜ [Rs, Cs],
///   x₀ ≜ Q₀, dx/dt = f(t, x; 𝐤) = (E(t) - x / 𝐤₂) / 𝐤₁.
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

  /// Scalar ODE dx/dt = f(t, x; 𝐤) function type.
  ///
  /// @param t The independent variable t ∈ ℝ .
  /// @param x The dependent variable x ∈ ℝ .
  /// @param k The parameter vector 𝐤 ∈ ℝᵐ.
  /// @return The derivative dx/dt ∈ ℝ.
  typedef std::function<T(const T& t, const T& x,
                          const VectorX<T>& k)> ScalarODEFunction;

  /// Constructs a scalar IVP described by the given @p scalar_ode_function,
  /// using the given @p default_initial_time and @p default_initial_state as
  /// initial conditions, and parameterized with @p default_parameters by
  /// default. Defaults are later used to solve the IVP when the corresponding
  /// argument is left unspecified.
  ///
  /// @param scalar_ode_function The scalar ODE function f(t, x; 𝐤) that
  ///                            describes the state evolution over time.
  /// @param default_initial_time The default initial time t₀.
  /// @param default_initial_state The default initial state x₀.
  /// @param default_parameters The default parameter vector 𝐤₀.
  ScalarInitialValueProblem(const ScalarODEFunction& scalar_ode_function,
                            const T& default_initial_time,
                            const T& default_initial_state,
                            const VectorX<T>& default_parameters);

  /// Solves the IVP for the given time @p tf, using the default parameters
  /// vector 𝐤₀ and default initial conditions (t₀, x₀) specified at
  /// construction.
  ///
  /// @param tf The time to solve the IVP for.
  /// @return The IVP solution x(@p tf; 𝐤) for x(t₀; 𝐤₀) = x₀.
  /// @pre The time @p tf must be larger than or equal to the default initial
  ///      time t₀ specified at construction.
  /// @throw std::logic_error if preconditions are not met.
  inline T Solve(const T& tf) const {
    const VectorX<T> solution =
        this->vector_ivp_->Solve(tf);
    return solution[0];
  }

  /// Solves the IVP for the given time @p tf, starting at the given initial
  /// time @p t0, using default parameter vector 𝐤₀ and default initial state
  /// x₀ specified at construction.
  ///
  /// @param t0 The initial time for the IVP.
  /// @param tf The time to solve the IVP for.
  /// @return The IVP solution x(@p tf; 𝐤₀) for x(@p t0; 𝐤₀) = x₀.
  /// @pre The time @p tf must be larger than or equal to the initial time
  ///      @p t0.
  /// @throw std::logic_error if preconditions are not met.
  inline T Solve(const T& t0, const T& tf) const {
    const VectorX<T> solution =
        this->vector_ivp_->Solve(t0, tf);
    return solution[0];
  }

  /// Solves the IVP for the given time @p tf using the given parameter vector
  /// @p k and default initial conditions (t₀, x₀) specified at construction.
  ///
  /// @param tf The time to solve the IVP for.
  /// @param k The parameters vector for the IVP.
  /// @return The IVP solution x(@p tf; @p k) for x(t₀; @p k) = x₀.
  /// @pre The time @p tf must be larger than or equal to the default initial
  ///      time t₀ specified at construction.
  /// @pre The dimension of @p k must match that of the default parameters
  ///      vector 𝐤₀ specified at construction.
  /// @throw std::logic_error if preconditions are not met.
  inline T Solve(const T& tf, const VectorX<T>& k) const {
    const VectorX<T> solution =
        this->vector_ivp_->Solve(tf, k);
    return solution[0];
  }

  /// Solves the IVP for the given time @p tf using the given parameter
  /// vector @p k and starting at time @p t0 with the default initial state
  /// x₀ specified at construction.
  ///
  /// @param t0 The initial time for the IVP.
  /// @param tf The time to solve the IVP for.
  /// @param k The parameter vector for the IVP.
  /// @return The IVP solution x(@p tf; @p k) for x(@p t0; @p k) = x₀.
  /// @pre The time @p tf must be larger than or equal to the initial time
  ///      @p t0.
  /// @pre The dimension of @p k must match that of the default parameter
  ///      vector 𝐤₀ given on construction.
  /// @throw std::logic_error if preconditions are not met.
  inline T Solve(const T& t0, const T& tf, const VectorX<T>& k) const {
    const VectorX<T> solution =
        this->vector_ivp_->Solve(t0, tf, k);
    return solution[0];
  }

  /// Solves the IVP for the given time @p tf using the given parameter
  /// vector @p k and given initial conditions (@p t0, @p x0).
  ///
  /// @param t0 The initial time for the IVP.
  /// @param tf The time to solve the IVP for.
  /// @param x0 The initial state of the IVP.
  /// @param k The parameter vector for the IVP.
  /// @return The IVP solution x(@p tf; @p k) for x(@p t0; @p k) = @p x0.
  /// @pre The time @p tf must be larger than or equal to the initial time
  ///      @p t0.
  /// @pre The dimension of @p k must match that of the default parameter
  ///      vector 𝐤₀ specified at construction.
  /// @throw std::logic_error if preconditions are not met.
  inline T Solve(const T& t0, const T& x0,
                 const T& tf, const VectorX<T>& k) const {
    const VectorX<T> solution = this->vector_ivp_->Solve(
        t0, VectorX<T>::Constant(1, x0), tf, k);
    return solution[0];
  }

  /// Resets the internal integrator instance.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    scalar_ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @return The new integrator instance.
  /// @tparam I The integrator type, which must be an IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          ScalarInitialValueProblem::get_integrator() and
  ///          ScalarInitialValueProblem::get_mutable_integrator().
  template <typename I, typename... Args>
  inline I* reset_integrator(Args&&... args) {
    return vector_ivp_->template reset_integrator(
        std::forward<Args>(args)...);
  }

  /// Gets a pointer to the internal integrator instance.
  inline const IntegratorBase<T>* get_integrator() const {
    return vector_ivp_->get_integrator();
  }

  /// Gets a pointer to the internal mutable integrator instance.
  inline IntegratorBase<T>* get_mutable_integrator() {
    return vector_ivp_->get_mutable_integrator();
  }

 private:
  // Vector IVP representation of this scalar IVP.
  std::unique_ptr<InitialValueProblem<T>> vector_ivp_;
};

}  // namespace systems
}  // namespace drake
