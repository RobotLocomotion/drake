#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/parameters.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// A general initial value problem (or IVP) representation class, that allows
/// evaluating the 𝐱(t; 𝐤) solution function to the given ODE
/// d𝐱/dt = f(t, 𝐱; 𝐤), where f : t ⨯ 𝐱 → ℝⁿ, t ∈ ℝ, 𝐱 ∈ ℝⁿ, 𝐤 ∈ ℝᵐ,
/// provided an initial condition 𝐱(t₀; 𝐤) = 𝐱₀. The parameter vector 𝐤
/// allows for generic IVP definitions, which can later be solved for any
/// instance of said vector.
///
/// Additionally, basic support to compute approximating functions for the IVP
/// solution is provided. This is convenient when throughput and efficiency are
/// of paramount importance.
///
/// This class' implementation performs basic computation caching, optimizing
/// away repeated integration whenever the IVP is solved for increasing values
/// of time t while both initial conditions and parameters are kept constant,
/// e.g. if solved for t₁ > t₀ first, solving for t₂ > t₁ will only require
/// integrating from t₁ onward.
///
/// For further insight into its use, consider the following examples:
///
/// - The momentum 𝐩 of a particle of mass m that is traveling through a
///   volume of a gas with dynamic viscosity μ can be described by
///   d𝐩/dt = -μ * 𝐩/m. At time t₀, the particle carries an initial momentum
///   𝐩₀. In this context, t is unused (the ODE is autonomous), 𝐱 ≜ 𝐩,
///   𝐤 ≜ [m, μ], t₀ = 0, 𝐱₀ ≜ 𝐩₀, d𝐱/dt = f(t, 𝐱; 𝐤) = -k₂ * 𝐱 / k₁.
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
  /// @param t The independent scalar variable t ∈ ℝ.
  /// @param x The dependent vector variable 𝐱 ∈ ℝⁿ.
  /// @param k The vector of parameters 𝐤 ∈ ℝᵐ.
  /// @return The derivative vector d𝐱/dt ∈ ℝⁿ.
  using ODEFunction = std::function<VectorX<T> (
      const T& t, const VectorX<T>& x, const VectorX<T>& k)>;

  /// Approximation technique function type, to build an approximating function
  /// 𝐳(t) to an 𝐱(t; 𝐤) solution based on a partition of the solution domain
  /// for which value 𝐱 and first derivative d𝐱/dt are known and provided at
  /// multiple argument values t.
  ///
  /// @param t_sequence The independent scalar variable sequence
  ///                   (t₁ ... tₚ) where tₚ ∈ ℝ sequence.
  /// @param x_sequence The dependent vector variable sequence
  ///                   (𝐱(t₁) ... 𝐱(tₚ)) where 𝐱(tₚ) ∈ ℝⁿ.
  /// @param dxdt_sequence The dependent vector variable first derivative
  ///                      sequence ((d𝐱/dt)(t₁) ... (d𝐱/dt)(tₚ)) where
  ///                      (d𝐱/dt)(tₚ) ∈ ℝⁿ.
  /// @return The approximating function 𝐳(t) ∈ ℝⁿ.
  /// @tparam ApproximatingFn The approximating function type.
  template <typename ApproximatingFn>
  using ApproximationTechnique = std::function<ApproximatingFn (
      const std::vector<T>& t_sequence,
      const std::vector<VectorX<T>>& x_sequence,
      const std::vector<VectorX<T>>& dxdt_sequence)>;

  /// A collection of values i.e. initial time t₀, initial state vector 𝐱₀
  /// and parameters vector 𝐤.to further specify the ODE system (in order
  /// to become an initial value problem).
  struct SpecifiedValues {
    /// Default constructor, leaving all values unspecified.
    SpecifiedValues() = default;

    /// Constructor specifying all values.
    ///
    /// @param t0_in Specified initial time t₀.
    /// @param x0_in Specified initial state vector 𝐱₀.
    /// @param k_in Specified parameter vector 𝐤.
    SpecifiedValues(const optional<T>& t0_in,
                    const optional<VectorX<T>>& x0_in,
                    const optional<VectorX<T>>& k_in)
        : t0(t0_in), x0(x0_in), k(k_in) {}

    bool operator==(const SpecifiedValues& rhs) const {
      return (t0 == rhs.t0 && x0 == rhs.x0 && k == rhs.k);
    }

    bool operator!=(const SpecifiedValues& rhs) const {
      return (t0 != rhs.t0 || x0 != rhs.x0 || k != rhs.k);
    }

    optional<T> t0;  ///< The initial time t₀ for the IVP.
    optional<VectorX<T>> x0;  ///< The initial state vector 𝐱₀ for the IVP.
    optional<VectorX<T>> k;  ///< The parameter vector 𝐤 for the IVP.
  };

  /// Constructs an IVP described by the given @p ode_function, using
  /// given @p default_values.t0 and @p default_values.x0 as initial
  /// conditions, and parameterized with @p default_values.k by default.
  ///
  /// @param ode_function The ODE function f(t, 𝐱; 𝐤) that describes the state
  ///                     evolution over time.
  /// @param default_values The values specified by default for this IVP, i.e.
  ///                       default initial time t₀ ∈ ℝ and state vector
  ///                       𝐱₀ ∈ ℝⁿ, and default parameter vector 𝐤 ∈ ℝᵐ.
  /// @pre An initial time @p default_values.t0 is given.
  /// @pre An initial state vector @p default_values.x0 is given.
  /// @pre A parameter vector @p default_values.k is given.
  /// @throw std::logic_error if preconditions are not met.
  InitialValueProblem(const ODEFunction& ode_function,
                      const SpecifiedValues& default_values);

  /// Solves the IVP for time @p tf, using the initial time t₀, initial state
  /// vector 𝐱₀ and parameter vector 𝐤 present in @p values, falling back to
  /// the ones given on construction if not given.
  ///
  /// @param tf The time to solve the IVP for.
  /// @param values The specified values for the IVP.
  /// @return The IVP solution 𝐱(@p tf; 𝐤) for 𝐱(t₀; 𝐤) = 𝐱₀.
  /// @pre Given @p tf must be larger than or equal to the specified initial
  ///      time t₀ (either given or default).
  /// @pre If given, the dimension of the initial state vector @p values.x0
  ///      must match that of the default initial state vector in the default
  ///      specified values given on construction.
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector in the default specified
  ///      values given on construction.
  /// @throw std::logic_error if preconditions are not met.
  VectorX<T> Solve(const T& tf, const SpecifiedValues& values = {}) const;

  /// Approximates the IVP solution 𝐱(t; 𝐤) with 𝐱(t₀; 𝐤) = 𝐱₀ using
  /// another function 𝐳(t; 𝐤) defined for t₀ <= t <= @p tf using the
  /// given @p approximation_technique.
  ///
  /// To this end, the domain interval [t₀, @p tf] is partitioned. Both
  /// 𝐱 and d𝐱/dt values are then solved and computed respectively for the
  /// boundaries of each sub-interval to feed the @p approximation_technique
  /// (see ApproximationTechnique's documentation).
  ///
  /// Implementation wise, above's procedure is equivalent to repeatedly
  /// calling the solver (see InitialValueProblem::Solve() method) and
  /// the ODE function (see InitialValueProblem::ODEFunction) specified
  /// on construction, iterating over the domain interval with an
  /// appropriate step. In this case, however, step selection is delegated
  /// to the integrator. This allows, depending on the integrator type, for
  /// fixed step computations with reduced overhead and error controlled
  /// step sizes, particularly helpful when an error bounded dense output
  /// [Hairer, 1993] is to be generated.
  ///
  /// - [Hairer, 1993] E. Hairer, S. Nørsett and G. Wanner. Solving Ordinary
  ///                  Differential Equations I (Nonstiff Problems), p. 190.
  ///                  Springer, 1993.
  ///
  /// @param approximation_technique The technique that builds the
  /// approximating function 𝐳(t; 𝐤) out of the provided partition.
  /// @param tf The time to approximate the IVP up to.
  /// @param values The specified values for the IVP.
  /// @return The approximation 𝐳(t; 𝐤) to the IVP solution 𝐱(t; 𝐤)
  ///         with 𝐱(t₀; 𝐤) = 𝐱₀ for t₀ <= t <= @p tf.
  /// @tparam ApproximatingFn The approximating function type.
  /// @pre Given @p tf must be larger than or equal to the specified initial
  ///      time t₀ (either given or default).
  /// @pre If given, the dimension of the initial state vector @p values.x0
  ///      must match that of the default initial state vector in the default
  ///      specified values given on construction.
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector in the default specified
  ///      values given on construction.
  /// @throw std::logic_error if preconditions are not met.
  template <typename ApproximatingFn>
  ApproximatingFn Approximate(
      const ApproximationTechnique<ApproximatingFn>& approximation_technique,
      const T& tf, const SpecifiedValues& values = {}) const;

  /// Resets the internal integrator instance by in-place
  /// construction of the given integrator type.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @return The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///         IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          InitialValueProblem::get_integrator() and
  ///          InitialValueProblem::get_mutable_integrator().
  template <typename Integrator, typename... Args>
  Integrator* reset_integrator(Args&&... args) {
    integrator_ = std::make_unique<Integrator>(
        *system_, std::forward<Args>(args)...);
    integrator_->reset_context(context_.get());
    return static_cast<Integrator*>(integrator_.get());
  }

  /// Gets a pointer to the internal integrator instance.
  const IntegratorBase<T>* get_integrator() const {
    return integrator_.get();
  }

  /// Gets a pointer to the internal mutable integrator instance.
  IntegratorBase<T>* get_mutable_integrator() {
    return integrator_.get();
  }

 private:
  // Sanitizes given @p values to solve for @p tf, i.e. sets defaults
  // when values are missing and validates that all preconditions specified
  // for InitialValueProblem::Solve() and InitialValueProblem::Approximate()
  // hold.
  //
  // @param tf The time to solve the IVP for.
  // @param values The specified values for the IVP.
  // @return Sanitized values.
  // @throw std::logic_error If preconditions specified for
  //                         InitialValueProblem::Solve() and
  //                         InitialValueProblem::Approximate()
  //                         do not hold.
  SpecifiedValues SanitizeValuesOrThrow(
      const T& tf, const SpecifiedValues& values) const;

  // IVP values specified by default.
  const SpecifiedValues default_values_;

  // @name Caching support
  //
  // In order to provide basic computation caching, both cache
  // initialization and cache invalidation must occur on IVP
  // solution evaluation. The mutability of the cached results
  // (and the conditions that must hold for them to be valid)
  // expresses the fact that neither computation results nor IVP
  // definition are affected when these change.

  // Invalidates and initializes cached IVP specified values and
  // integration context based on the newly provided @p values.
  void ResetCachedState(const SpecifiedValues& values) const;

  // Conditionally invalidates and initializes cached IVP specified
  // values and integration context based on time @p tf to solve for
  // and the provided @p values. If cached state can be reused, it's a
  // no-op.
  void MayResetCachedState(const T& tf, const SpecifiedValues& values) const;

  // IVP current specified values (for caching).
  mutable SpecifiedValues current_values_;

  // IVP ODE solver integration context.
  std::unique_ptr<Context<T>> context_;
  // IVP system representation used for ODE solving.
  std::unique_ptr<System<T>> system_;
  // Numerical integrator used for IVP ODE solving.
  std::unique_ptr<IntegratorBase<T>> integrator_;
};

}  // namespace systems
}  // namespace drake
