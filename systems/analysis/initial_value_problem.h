#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// A general initial value problem (or IVP) representation class, that allows
/// for the evaluation of the 𝐱(t; 𝐤) solution function to the given ODE
/// d𝐱/dt = f(t, 𝐱; 𝐤), where f : t ⨯ 𝐱 → ℝⁿ, t ∈ ℝ, 𝐱 ∈ ℝⁿ, 𝐤 ∈ ℝᵐ, provided an
/// initial condition 𝐱(t₀; 𝐤) = 𝐱₀. The parameter vector 𝐤 allows for generic
/// IVP definitions, particularized on evaluation.
///
/// For further insight into its use, consider the following examples:
///
/// - The momentum 𝐩 of a particle of mass m that is travelling through a
///   volume of a gas with dynamic viscosity μ can be described by
///   d𝐩/dt = -μ * 𝐩/m. At time t₀, the particle carries an initial momentum
///   𝐩₀. In this context, t is unused (the ODE is autonomous), 𝐱 ≜ 𝐩,
///   𝐤 ≜ [m, μ], t₀ = 0, 𝐱₀ ≜  𝐩₀, d𝐱/dt = f(t, 𝐱; 𝐤) = -k₂ * 𝐱 / k₁.
///
/// - The velocity 𝐯 of the same particle in the same exact conditions as
///   before, but when a time varying force 𝐅(t) is applied to it, can be
///   be described by d𝐯/dt = (𝐅(t) - μ * 𝐯) / m. In this context, 𝐱 ≜ 𝐯,
///   𝐤 ≜ [m, μ], 𝐱₀ ≜ 𝐯₀, d𝐱/dt = f(t, 𝐱; 𝐤) = (𝐅(t) - k₂ * 𝐱) / k₁.
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

  /// General ODE system d𝐱/dt = f(t, 𝐱; 𝐤) function type.
  ///
  /// @param t The independent scalar variable t ∈ ℝ .
  /// @param x The dependent vector variable 𝐱 ∈ ℝⁿ.
  /// @param k The vector of parameters 𝐤 ∈ ℝᵐ.
  /// @return The derivative vector d𝐱/dt ∈ ℝⁿ.
  typedef std::function<VectorX<T> (
      const T& t, const VectorX<T>& x,
      const VectorX<T>& k)> ODEFunction;

  /// Constructs an IVP described by the given @p ode_function, using
  /// given @p default_initial_time and @p default_initial_state as initial
  /// conditions, and parameterized with @p default_parameters by default.
  ///
  /// @param ode_function The ODE function f(t, 𝐱; 𝐤) that describes the state
  ///                     evolution over time.
  /// @param default_initial_time The default initial time t₀ ∈ ℝ.
  /// @param default_initial_state The default initial state vector 𝐱₀ ∈ ℝⁿ.
  /// @param default_parameters The default parameters vector 𝐤₀ ∈ ℝᵐ.
  InitialValueProblem(const ODEFunction& ode_function,
                      const T& default_initial_time,
                      const VectorX<T>& default_initial_state,
                      const VectorX<T>& default_parameters);

  /// Solves the IVP for the given time @p t , using default parameters vector
  /// 𝐤₀ and default initial conditions (t₀, 𝐱₀).
  ///
  /// @param t The time t to solve the IVP for.
  /// @return The IVP solution 𝐱(@p t ; 𝐤₀) for 𝐱(t₀ ; 𝐤₀) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  ///      given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline VectorX<T> Solve(const T& t) const {
    return this->Solve(default_initial_time_, t);
  }

  /// Solves the IVP for the given time @p t , starting at the given initial
  /// time @p t0 , and using default parameters vector 𝐤₀ and default initial
  /// state vector 𝐱₀.
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP for.
  /// @return The IVP solution 𝐱(@p t ; 𝐤₀) for 𝐱(@p t0 ; 𝐤₀) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  ///      @p t0 .
  /// @warning This method will abort if preconditions are not met.
  inline VectorX<T> Solve(const T& t0, const T& t) const {
    return this->Solve(t0, t, default_parameters_);
  }

  /// Solves the IVP for the given time @p t with default initial conditions
  /// (t₀, 𝐱₀) and using the given parameters vector @p k .
  ///
  /// @param t The time to solve the IVP for.
  /// @param k The parameters vector for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p k ) for 𝐱(t₀; @p k ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time t₀
  ///      given on construction.
  /// @pre The dimension of the given parameters vector @p k must match that
  ///      of the default parameters vector 𝐤₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline VectorX<T> Solve(const T& t, const VectorX<T>& k) const {
    return this->Solve(default_initial_time_, t, k);
  }

  /// Solves the IVP starting at time @p t0 with default initial state vector
  /// 𝐱₀ for the given time @p t and using the given parameters @p k .
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP for.
  /// @param k The parameters vector for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p k ) for 𝐱(@p t0 ; @p k ) = 𝐱₀.
  /// @pre The time @p t must be larger than or equal to the initial time
  ///      @p t0 .
  /// @pre The dimension of the given parameters vector @p k must match that
  ///      of the default parameters vector 𝐤₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline VectorX<T> Solve(const T& t0, const T& t,
                          const VectorX<T>& k) const {
    return this->Solve(t0, default_initial_state_, t, k);
  }

  /// Solves the IVP starting at time @p t0 with default initial state @p x0
  /// for the given time @p t , using the given parameters @p k .
  ///
  /// @param t0 The initial time for the IVP.
  /// @param t The time to solve the IVP for.
  /// @param x0 The initial state vector of the IVP.
  /// @param k The parameter vector for the IVP.
  /// @return The IVP solution 𝐱(@p t ; @p k ) for 𝐱(@p t0 ; @p k ) = @p x0 .
  /// @pre The time @p t must be larger than or equal to the initial time @p t0.
  /// @pre The dimension of the given initial state @p x0 must match that of the
  ///      default initial state vector 𝐱₀ given on construction.
  /// @pre The dimension of the given parameters vector @p k must match that
  ///      of the default parameters vector 𝐤₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  VectorX<T> Solve(const T& t0, const VectorX<T>& x0,
                   const T& t, const VectorX<T>& k) const;

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
  // IVP default initial time t₀.
  const T default_initial_time_;
  // IVP default initial state 𝐱₀.
  const VectorX<T> default_initial_state_;
  // IVP default parameters 𝐤₀.
  const VectorX<T> default_parameters_;
  // IVP current initial time tᵢ (for caching).
  mutable T current_initial_time_;
  // IVP current initial state xᵢ (for caching).
  mutable VectorX<T> current_initial_state_;
  // IVP current parameters 𝐤ᵢ(for caching).
  mutable VectorX<T> current_parameters_;
  // IVP ODE solver integration context (for caching).
  mutable std::unique_ptr<Context<T>> context_;
  // IVP system representation used for ODE solving.
  std::unique_ptr<System<T>> system_;
  // Numerical integrator used for IVP ODE solving.
  std::unique_ptr<IntegratorBase<T>> integrator_;
};

}  // namespace systems
}  // namespace drake
