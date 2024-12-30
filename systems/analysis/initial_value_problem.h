#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/dense_output.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

/// A general initial value problem (or IVP) representation class, that allows
/// evaluating the 𝐱(t; 𝐤) solution function to the given ODE
/// d𝐱/dt = f(t, 𝐱; 𝐤), where f : t ⨯ 𝐱 → ℝⁿ, t ∈ ℝ, 𝐱 ∈ ℝⁿ, 𝐤 ∈ ℝᵐ,
/// provided an initial condition 𝐱(t₀; 𝐤) = 𝐱₀. The parameter vector 𝐤
/// allows for generic IVP definitions, which can later be solved for any
/// instance of said vector.
///
/// By default, an explicit 3rd order RungeKutta integration scheme is used.
///
/// The implementation of this class performs basic computation caching,
/// optimizing away repeated integration whenever the IVP is solved for
/// increasing values of time t while both initial conditions and parameters
/// are kept constant, e.g. if solved for t₁ > t₀ first, solving for t₂ > t₁
/// will only require integrating from t₁ onward.
///
/// Additionally, IntegratorBase's dense output support can be leveraged to
/// efficiently approximate the IVP solution within closed intervals of t.
/// This is convenient when there's a need for a more dense sampling of the
/// IVP solution than what would be available through either fixed or
/// error-controlled step integration (for a given accuracy), or when the IVP
/// is to be solved repeatedly for arbitrarily many t values within a given
/// interval. See documentation of the internally held IntegratorBase subclass
/// instance (either the default or a user-defined one, set via
/// reset_integrator()) for further reference on the specific dense output
/// technique in use.
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
/// @tparam_nonsymbolic_scalar
template <typename T>
class InitialValueProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InitialValueProblem);

  /// Default integration accuracy in the relative tolerance sense.
  static const double kDefaultAccuracy;
  /// Default initial integration step size.
  static const double kInitialStepSize;
  /// Default maximum integration step size.
  static const double kMaxStepSize;

  /// General ODE system d𝐱/dt = f(t, 𝐱; 𝐤) function type.
  ///
  /// @param t The independent scalar variable t ∈ ℝ.
  /// @param x The dependent vector variable 𝐱 ∈ ℝⁿ.
  /// @param k The vector of parameters 𝐤 ∈ ℝᵐ.
  /// @return The derivative vector d𝐱/dt ∈ ℝⁿ.
  using OdeFunction = std::function<VectorX<T>(const T& t, const VectorX<T>& x,
                                               const VectorX<T>& k)>;

  /// Constructs an IVP described by the given @p ode_function, using @p x0 as
  /// initial conditions, and parameterized with @p k.
  ///
  /// @param ode_function The ODE function f(t, 𝐱; 𝐤) that describes the state
  ///                     evolution over time.
  /// @param x0 The initial state vector 𝐱₀ ∈ ℝⁿ.
  /// @param k The parameter vector 𝐤 ∈ ℝᵐ.  By default m=0 (no parameters).
  InitialValueProblem(const OdeFunction& ode_function,
                      const Eigen::Ref<const VectorX<T>>& x0,
                      const Eigen::Ref<const VectorX<T>>& k = Vector0<T>{});

  /// Solves the IVP from the initial time @p t0 up to time @p tf, using the
  /// initial state vector 𝐱₀ and parameter vector 𝐤 provided in the
  /// constructor.
  /// @throws std::exception if t0 > tf.
  VectorX<T> Solve(const T& t0, const T& tf) const;

  /// Solves and yields an approximation of the IVP solution x(t; 𝐤) for the
  /// closed time interval between the given initial time @p t0 and the given
  /// final time @p tf, using initial state 𝐱₀ and parameter vector 𝐤 provided
  /// in the constructor.
  ///
  /// To this end, the wrapped IntegratorBase instance solves this IVP,
  /// advancing time and state from t₀ and 𝐱₀ = 𝐱(t₀) to @p tf and 𝐱(@p tf),
  /// creating a dense output over that [t₀, @p tf] interval along the way.
  ///
  /// @param tf The IVP will be solved up to this time, which must be ≥ t₀.
  /// Usually, t₀ < @p tf as an empty dense output would result if t₀ = @p tf.
  /// @returns A dense approximation to 𝐱(t; 𝐤) with 𝐱(t₀; 𝐤) = 𝐱₀,
  /// defined for t₀ ≤ t ≤ tf.
  /// @note The larger the given @p tf value is, the larger the approximated
  ///       interval will be. See documentation of the specific dense output
  ///       technique in use for reference on performance impact as this
  ///       interval grows.
  /// @throws std::exception if t0 > tf.
  std::unique_ptr<DenseOutput<T>> DenseSolve(const T& t0, const T& tf) const;

  /// Resets the internal integrator instance by in-place
  /// construction of the given integrator type.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @returns The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///         IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          InitialValueProblem::get_integrator() and
  ///          InitialValueProblem::get_mutable_integrator().
  template <typename Integrator, typename... Args>
  Integrator* reset_integrator(Args&&... args) {
    integrator_ =
        std::make_unique<Integrator>(*system_, std::forward<Args>(args)...);
    integrator_->reset_context(context_.get());
    return static_cast<Integrator*>(integrator_.get());
  }

  /// Gets a reference to the internal integrator instance.
  const IntegratorBase<T>& get_integrator() const {
    DRAKE_DEMAND(integrator_ != nullptr);
    return *integrator_.get();
  }

  /// Gets a mutable reference to the internal integrator instance.
  IntegratorBase<T>& get_mutable_integrator() {
    DRAKE_DEMAND(integrator_ != nullptr);
    return *integrator_.get();
  }

 private:
  // Resets the context / integrator between multiple solves.
  void ResetState() const;

  // IVP ODE solver integration context.
  std::unique_ptr<Context<T>> context_;
  // IVP system representation used for ODE solving.
  std::unique_ptr<System<T>> system_;
  // Numerical integrator used for IVP ODE solving.
  std::unique_ptr<IntegratorBase<T>> integrator_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::InitialValueProblem);
