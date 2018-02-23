#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// A general initial value problem (or IVP) representation class, that allows
/// for the evaluation of the 𝐱(t; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) solution function to the
/// given ODE d𝐱/dt = f(t, 𝐱; 𝐤₀, 𝐤₁, ..., 𝐤ₙ), where f : t ⨯ 𝐱 → ℝⁿ, t ∈ ℝ,
/// 𝐱 ∈ ℝⁿ, 𝐤₀ ∈ ℝᵐ⁰, 𝐤₁ ∈ ℝᵐ¹, ..., 𝐤ₙ ∈ ℝᵐᶻ, provided an initial
/// condition 𝐱(t₀; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) = 𝐱₀. The n-tuple of vector parameters
/// 𝐤₀, 𝐤₁, ..., 𝐤ₙ allows for generic IVP definitions, particularized on
/// evaluation.
///
/// For further insight into its use, consider the following examples:
///
/// - The momentum 𝐩 of a particle of mass m that is travelling through a
///   volume of a gas with dynamic viscosity μ can be described by
///   d𝐩/dt = -μ * 𝐩/m. At time t₀, the particle carries an initial momentum
///   𝐩₀. In this context, t is unused (the ODE is autonomous), 𝐱 ≜ 𝐩,
///   𝐤₀ ≜ [m, μ], t₀ = 0, 𝐱₀ ≜  𝐩₀, d𝐱/dt = f(t, 𝐱; 𝐤₀) = -k₀₂ * 𝐱 / k₀₁.
///
/// - The velocity 𝐯 of the same particle in the same exact conditions as
///   before, but when a time varying force 𝐅(t) is applied to it, can be
///   be described by d𝐯/dt = (𝐅(t) - μ * 𝐯) / m. In this context, 𝐱 ≜ 𝐯,
///   𝐤₀ ≜ [m, μ], 𝐱₀ ≜ 𝐯₀, d𝐱/dt = f(t, 𝐱; 𝐤₀) = (𝐅(t) - k₀₂ * 𝐱) / k₀₁.
///
/// @tparam T The ℝ domain scalar type, which must be a valid Eigen scalar.
///
/// @note
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
template <typename T>
class InitialValueProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InitialValueProblem);

  /// Default integration accuracy in the relative tolerance sense.
  static const T kDefaultAccuracy;
  /// Default initial integration step size.
  static const T kInitialStepSize;
  /// Default maximum integration step size.
  static const T kMaxStepSize;

  /// General ODE system d𝐱/dt = f(t, 𝐱; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) function type.
  ///
  /// @param t The independent scalar variable t ∈ ℝ .
  /// @param x The dependent vector variable 𝐱 ∈ ℝⁿ.
  /// @param k The parameters 𝐤₀ ∈ ℝᵐ⁰, 𝐤₁ ∈ ℝᵐ¹, ..., 𝐤ₙ ∈ ℝᵐᶻ.
  /// @param[out] dx_dt The derivative vector d𝐱/dt∈ ℝⁿ.
  typedef std::function<void(
      const T& t, const VectorBase<T>& x,
      const Parameters<T>& k, VectorBase<T>* dx_dt)> ODEFunction;

  /// Constructs an IVP described by the given @p ode_function, using
  /// given @p default_initial_time and @p default_initial_state as initial
  /// conditions, and parameterized with @p default_parameters by default.
  ///
  /// @param ode_function The ODE function f(t, 𝐱; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) that
  /// describes the state evolution over time.
  /// @param default_initial_time The default initial time t₀.
  /// @param default_initial_state The default initial state 𝐱₀.
  /// @param default_parameters The default parameters 𝐤₀ ∈ ℝᵐ⁰,
  /// 𝐤₁ ∈ ℝᵐ¹, ..., 𝐤ₙ ∈ ℝᵐᶻ.
  InitialValueProblem(const ODEFunction& ode_function,
                      const T& default_initial_time,
                      const BasicVector<T>& default_initial_state,
                      const Parameters<T>& default_parameters);

  /// Solves the IVP for the given time @p t , using default parameters
  /// 𝐤₀, 𝐤₁, ..., 𝐤ₙ and default initial conditions (t₀, 𝐱₀).
  ///
  /// @param t The time t to solve the IVP for.
  /// @return The IVP solution 𝐱(@p t ; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) for
  ///         𝐱(t₀ ; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  ///      given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t) const {
    const T& default_initial_time =
        default_context_->get_time();
    return this->Solve(default_initial_time, t);
  }

  /// Solves the IVP for the given time @p t , starting at the given initial
  /// time @p t0 , and using default parameters 𝐤₀, 𝐤₁, ..., 𝐤ₙ and default
  /// initial state 𝐱₀.
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP for.
  /// @return The IVP solution 𝐱(@p t ; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) for
  ///         𝐱(@p t0 ; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  ///      @p t0 .
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t0, const T& t) const {
    const Parameters<T>& default_parameters =
        default_context_->get_parameters();
    return this->Solve(t0, t, default_parameters);
  }

  /// Solves the IVP for the given time @p t with default initial conditions
  /// and using the given parameters @p k .
  ///
  /// @param t The time to solve the IVP for.
  /// @param k The parameters for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p k ) for 𝐱(t₀; @p k ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  ///      given on construction.
  /// @pre The length and order of the given parameters @p k, and the dimension
  ///      of each of its elements must match that of the default parameters
  ///      𝐤₀, 𝐤₁, ..., 𝐤ₙ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t, const Parameters<T>& k) const {
    const T& default_initial_time =
        default_context_->get_time();
    return this->Solve(default_initial_time, t, k);
  }

  /// Solves the IVP starting at time @p t0 with default initial state
  /// for the given time @p t and using the given parameters @p k .
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP for.
  /// @param k The parameters for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p k ) for 𝐱(@p t0 ; @p k ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  ///      @p t0 .
  /// @pre The length and order of the given parameters @p k, and the dimension
  ///      of each of its elements must match that of the default parameters
  ///      𝐤₀, 𝐤₁, ..., 𝐤ₙ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t0, const T& t,
                                    const Parameters<T>& k) const {
    const VectorBase<T>& default_initial_state =
        default_context_->get_continuous_state_vector();
    return this->Solve(t0, default_initial_state, t, k);
  }

  /// Solves the IVP starting at time @p t0 with default initial state
  /// for the given time @p t using the given parameters @p k .
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP for.
  /// @param x0 The initial state of the IVP.
  /// @param k The parameters for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p k ) for 𝐱(@p t0 ; @p k ) = @p x0 .
  /// @pre The time @p t must be larger than or equal to the initial time @p t0.
  /// @pre The dimension of the given initial state @p x0 must match that of the
  ///      default initial state 𝐱₀ given on construction.
  /// @pre The length and order of the given parameters @p k, and the dimension
  ///      of each of its elements must match that of the default parameters
  ///      𝐤₀, 𝐤₁, ..., 𝐤ₙ given on construction.
  /// @warning This method will abort if preconditions are not met.
  const VectorBase<T>& Solve(const T& t0, const VectorBase<T>& x0,
                             const T& t, const Parameters<T>& k) const;

  /// Resets the internal integrator instance.
  /// @return The new integrator instance.
  /// @tparam I The integrator type, which must be an IntegratorBase subclass.
  template <typename I>
  IntegratorBase<T>* reset_integrator();

  inline const IntegratorBase<T>* get_integrator() const {
    return integrator_.get();
  }

  inline IntegratorBase<T>* get_mutable_integrator() {
    return integrator_.get();
  }

 private:
  // Checks whether a given pair of Context instances describing initial and
  // current integration context can be reused to solve the IVP for given
  // initial time @p t0 and state @p x0, for the given time @p t with the
  // given parameters @p k. This allows optimizing away integration context
  // re-initializations for successive, incremental solving of the same IVP
  // with the same set of values.
  //
  // @param initial_context Initial integration context to be checked.
  // @param current_context Current integration context to be checked.
  // @param t0 The IVP's initial condition time.
  // @param x0 The IVP's initial condition state.
  // @param t The time to solve the IVP for.
  // @param k The IVP parameters.
  // @return True if the same context can be used to solve the IVP, False
  //         otherwise.
  bool AreContextsValid(const Context<T>& initial_context,
                        const Context<T>& current_context,
                        const T& t0, const VectorBase<T>& x0,
                        const T& t, const Parameters<T>& k) const;

  // IVP ODE solver integration default context.
  std::unique_ptr<Context<T>> default_context_;
  // IVP ODE solver integration initial context.
  std::unique_ptr<Context<T>> initial_context_;
  // IVP ODE solver integration current context.
  mutable std::unique_ptr<Context<T>> context_;
  // IVP system representation used for ODE solving.
  std::unique_ptr<System<T>> system_;
  // Numerical integrator used for IVP ODE solving.
  std::unique_ptr<IntegratorBase<T>> integrator_;
};

}  // namespace systems
}  // namespace drake
