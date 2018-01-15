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
/// for the evaluation of the 𝐱(t; 𝐩) solution function to the given ODE
/// d𝐱/dt = f(t, 𝐱; 𝐩), where f : t ⨯ 𝐱 ⊆ ℝ ⁿ⁺¹ →  d𝐱/dt ⊆ ℝ ⁿ, t ∈ ℝ , 𝐱 ∈ ℝ ⁿ,
/// 𝐩 ∈ ℝ ᵐ, provided an initial condition 𝐱(t₀; 𝐩) = 𝐱₀.
///
/// For further insight into its use, consider the following examples:
///
/// - The momentum 𝐩 of a particle of mass m that is travelling through a
///   volume of a gas with dynamic viscosity μ can be described by
///   d𝐩/dt = -μ * 𝐩/m. At time t₀, the particle carries an initial momentum
///   𝐩₀. In this context, t is unused (the IVP is autonomous), 𝐱 ≜ 𝐩,
///   𝐩 ≜ [m, μ], t₀ = 0, 𝐱₀ ≜  𝐩₀, d𝐱/dt = f(t, 𝐱; 𝐩) = -p₂ * 𝐱/p₁.
///
/// - The velocity 𝐯 of the same particle in the same exact conditions as
///   before, but when a time varying force 𝐅(t) is applied to it, can be
///   be described by d𝐯/dt = (𝐅(t) - μ * 𝐯) / m. In this context, 𝐱 ≜ 𝐯,
///   𝐩 ≜ [m, μ], 𝐱₀ ≜ 𝐯₀, d𝐱/dt = f(t, 𝐱; 𝐩) = (𝐅(t) - p₂ * 𝐱) /p₁.
///
/// @tparam T The ℝ domain scalar type, which must be a valid scalar type.
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
  /// Default initial integration step size as it's adapted to meet
  /// accuracy requirements.
  static const T kInitialStepSize;
  /// Default maximum integration step size as it's adapted to meet
  /// accuracy requirements.
  static const T kMaxStepSize;

  /// General ODE system d𝐱/dt = f(t, 𝐱; 𝐩) function type.
  ///
  /// @param t The independent scalar variable t ∈ ℝ .
  /// @param x The dependent vector variable 𝐱 ∈ ℝ ⁿ.
  /// @param p The parameter vector 𝐩 ∈ ℝ ᵐ .
  /// @param[out] dx_dt The derivative vector d𝐱/dt ∈ ℝ ⁿ.
  typedef std::function<void(
      const T& t, const VectorBase<T>& x,
      const Parameters<T>& p, VectorBase<T>* dx_dt)> ODEFunction;

  /// Constructs an IVP described by the given @p ode_function, using
  /// given @p default_initial_time and @p default_initial_state as initial
  /// conditions, and parameterized with @p default_parameters by default.
  ///
  /// @param ode_function The ODE function f(t, 𝐱; 𝐩) that describes the state
  /// evolution over time.
  /// @param default_initial_time The default initial time t₀.
  /// @param default_initial_state The default initial state 𝐱₀.
  /// @param default_parameters The default parameters 𝐩₀.
  InitialValueProblem(const ODEFunction& ode_function,
                      const T& default_initial_time,
                      const BasicVector<T>& default_initial_state,
                      const Parameters<T>& default_parameters);

  /// Solves the IVP for the given time @p t , using default parameters 𝐩₀ and
  /// default initial conditions (t₀, 𝐱₀).
  ///
  /// @param t The time t to solve the IVP up to.
  /// @return The IVP solution 𝐱(@p t ; 𝐩₀) for 𝐱(t₀ ; 𝐩₀) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  /// given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t) const {
    const T& default_initial_time =
        default_context_->get_time();
    return this->Solve(default_initial_time, t);
  }

  /// Solves the IVP for the given time @p t , starting at the given initial
  /// time @p t0 , and using default parameters 𝐩₀ and default initial state 𝐱₀.
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP up to.
  /// @return The IVP solution 𝐱(@p t ; 𝐩₀) for 𝐱(@p t0 ; 𝐩₀) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  /// @p t0 .
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t0, const T& t) const {
    const Parameters<T>& default_parameters =
        default_context_->get_parameters();
    return this->Solve(t0, t, default_parameters);
  }

  /// Solves the IVP starting at time @p t0 with default initial state
  /// for the given time @p t using default parameters.
  ///
  /// @param t The time to solve the IVP up to.
  /// @param p The parameters for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p p ) for 𝐱(t₀; @p p ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  /// given on construction.
  /// @pre The quantity and dimension of the given parameters @p p must match
  /// that of the default parameters 𝐩₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t, const Parameters<T>& p) const {
    const T& default_initial_time =
        default_context_->get_time();
    return this->Solve(default_initial_time, t, p);
  }


  /// Solves the IVP starting at time @p t0 with default initial state
  /// for the given time @p t using the given parameters @p p .
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP up to.
  /// @param p The parameters for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p p ) for 𝐱(@p t0 ; @p p ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  /// @p t0 .
  /// @pre The quantity and dimension of the given parameters @p p must match
  /// that of the default parameters 𝐩₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline const VectorBase<T>& Solve(const T& t0, const T& t,
                                    const Parameters<T>& p) const {
    const VectorBase<T>& default_initial_state =
        default_context_->get_continuous_state_vector();
    return this->Solve(t0, default_initial_state, t, p);
  }

  /// Solves the IVP starting at time @p t0 with default initial state
  /// for the given time @p t using the given parameters @p p .
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP up to.
  /// @param x0 The initial state of the IVP.
  /// @param p The parameters for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p p ) for 𝐱(@p t0 ; @p p ) = @p x0 .
  /// @pre The time @p t must be larger than or equal to the initial time @p t0.
  /// @pre The dimension of the given initial state @p x0 must match that of the
  /// default initial state 𝐱₀ given on construction.
  /// @pre The quantity and dimension of the given parameters @p p must match
  /// that of the default parameters 𝐩₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  const VectorBase<T>& Solve(const T& t0, const VectorBase<T>& x0,
                             const T& t, const Parameters<T>& p) const;

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
  // initial time @p t0 and state @p x0, up to the given time @p t with the
  // given parameterization @p p. This allows to optimize away integration
  // context re-initializations for successive, incremental solving of the same
  // IVP with the same set of values.
  //
  // @param initial_context Initial integration context to be checked.
  // @param current_context Current integration context to be checked.
  // @param t0 The IVP's initial condition time.
  // @param x0 The IVP's initial condition state.
  // @param t The time to solve the IVP upto.
  // @param p The IVP parameters.
  // @return True if the same context can be used to solve the IVP, False
  // otherwise.
  bool AreContextsValid(const Context<T>& initial_context,
                        const Context<T>& current_context,
                        const T& t0, const VectorBase<T>& x0,
                        const T& t, const Parameters<T>& p) const;

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
