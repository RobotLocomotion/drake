#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/analysis/scalar_view_dense_output.h"

namespace drake {
namespace systems {

/// A thin wrapper of the InitialValueProblem class to provide a simple
/// interface when solving scalar initial value problems i.e. when evaluating
/// the x(t; 𝐤) solution function to the given ODE dx/dt = f(t, x; 𝐤),
/// where f : t ⨯ x →  ℝ , t ∈ ℝ, x ∈ ℝ, 𝐤 ∈ ℝᵐ, along with an initial
/// condition x(t₀; 𝐤) = x₀. The parameter vector 𝐤 allows for generic IVP
/// definitions, which can later be solved for any instance of said vector.
///
/// Note the distinction from general initial value problems where
/// f : t ⨯ 𝐱 → ℝⁿ and 𝐱 ∈ ℝⁿ, addressed by the class being wrapped. While
/// every scalar initial value problem could be written in vector form, this
/// wrapper keeps both problem definition and solution in their scalar form
/// with almost zero overhead, leading to clearer code if applicable.
/// Moreover, this scalar form facilitates single-dimensional quadrature
/// using methods for solving initial value problems.
///
/// See InitialValueProblem class documentation for information on caching
/// support and dense output usage for improved efficiency in scalar IVP
/// solving.
///
/// For further insight into its use, consider the following examples of scalar
/// IVPs:
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
/// @tparam_nonsymbolic_scalar
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
  using ScalarOdeFunction =
      std::function<T(const T& t, const T& x, const VectorX<T>& k)>;

  /// Constructs a scalar IVP described by the given @p scalar_ode_function,
  /// using given @p x0 as initial conditions, and parameterized with @p k.
  ///
  /// @param scalar_ode_function The ODE function f(t, 𝐱; 𝐤) that describes
  /// the state evolution over time. @param x0 The initial state 𝐱₀ ∈ ℝ.
  /// @param k The parameter vector 𝐤 ∈ ℝᵐ.  By default m=0 (no parameters).
  ScalarInitialValueProblem(
      const ScalarOdeFunction& scalar_ode_function, const T& x0,
      const Eigen::Ref<const VectorX<T>>& k = Vector0<T>{});

  /// Solves the IVP from time @p t0 up to time @p tf, using the initial state
  /// 𝐱₀ and parameter vector 𝐤 provided in the constructor.
  /// @throws std::exception if t0 > tf.
  T Solve(const T& t0, const T& tf) const;

  /// Solves and yields an approximation of the IVP solution x(t; 𝐤) for the
  /// closed time interval between the initial time @p t0 and the final time @p
  /// tf, using initial state 𝐱₀ and parameter vector 𝐤 provided in the
  /// constructor.
  ///
  /// To this end, the wrapped IntegratorBase instance solves this IVP,
  /// advancing time and state from t₀ and 𝐱₀ = 𝐱(@p t0) to @p tf and 𝐱(@p
  /// tf), creating a dense output over that [@p t0, @p tf] interval along the
  /// way.
  ///
  /// @param tf The IVP will be solved up to this time, which must be ≥ @p t0.
  /// Usually, @p t0 < @p tf as an empty dense output would result if @p t0 =
  /// @p tf.
  /// @returns A dense approximation to 𝐱(t; 𝐤) with 𝐱(t0; 𝐤) = 𝐱₀,
  /// defined for t0 ≤ t ≤ tf.
  /// @note The larger the given @p tf value is, the larger the approximated
  ///       interval will be. See documentation of the specific dense output
  ///       technique in use for reference on performance impact as this
  ///       interval grows.
  /// @throws std::exception if t0 > tf.
  std::unique_ptr<ScalarDenseOutput<T>> DenseSolve(const T& t0,
                                                   const T& tf) const;

  /// Resets the internal integrator instance by in-place
  /// construction of the given integrator type.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    scalar_ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @returns The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///                    IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          ScalarInitialValueProblem::get_integrator() and
  ///          ScalarInitialValueProblem::get_mutable_integrator().
  template <typename Integrator, typename... Args>
  Integrator* reset_integrator(Args&&... args) {
    return vector_ivp_->template reset_integrator<Integrator>(
        std::forward<Args>(args)...);
  }

  /// Gets a reference to the internal integrator instance.
  const IntegratorBase<T>& get_integrator() const {
    return vector_ivp_->get_integrator();
  }

  /// Gets a mutable reference to the internal integrator instance.
  IntegratorBase<T>& get_mutable_integrator() {
    return vector_ivp_->get_mutable_integrator();
  }

 private:
  // Vector IVP representation of this scalar IVP.
  std::unique_ptr<InitialValueProblem<T>> vector_ivp_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ScalarInitialValueProblem);
