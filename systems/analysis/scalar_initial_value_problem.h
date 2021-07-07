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

  /// A collection of values i.e. initial time t₀, initial state x₀
  /// and parameter vector 𝐤 to further specify the ODE system (in
  /// order to become a scalar initial value problem).
  struct ScalarOdeContext {
    /// Default constructor, leaving all values unspecified.
    ScalarOdeContext() = default;

    /// Constructor specifying all values.
    ///
    /// @param t0_in Specified initial time t₀.
    /// @param x0_in Specified initial state x₀.
    /// @param k_in Specified parameter vector 𝐤.
    ScalarOdeContext(const std::optional<T>& t0_in,
                     const std::optional<T>& x0_in,
                     const std::optional<VectorX<T>>& k_in)
        : t0(t0_in), x0(x0_in), k(k_in) {}

    std::optional<T> t0;          ///< The initial time t₀ for the IVP.
    std::optional<T> x0;          ///< The initial state x₀ for the IVP.
    std::optional<VectorX<T>> k;  ///< The parameter vector 𝐤 for the IVP.
  };

  /// Constructs an scalar IVP described by the given @p scalar_ode_function,
  /// using given @p default_values.t0 and @p default_values.x0 as initial
  /// conditions, and parameterized with @p default_values.k by default.
  ///
  /// @param scalar_ode_function The ODE function f(t, x; 𝐤) that describes the
  ///                            state evolution over time.
  /// @param default_values The values specified by default for this IVP, i.e.
  ///                       default initial time t₀ ∈ ℝ and state x₀ ∈ ℝ, and
  ///                       default parameter vector 𝐤 ∈ ℝᵐ.
  /// @pre An initial time @p default_values.t0 is provided.
  /// @pre An initial state @p default_values.x0 is provided.
  /// @pre An parameter vector @p default_values.k is provided.
  /// @throws std::exception if preconditions are not met.
  ScalarInitialValueProblem(const ScalarOdeFunction& scalar_ode_function,
                            const ScalarOdeContext& default_values) {
    // Wraps the given scalar ODE function as a vector ODE function.
    typename InitialValueProblem<T>::OdeFunction ode_function =
        [scalar_ode_function](const T& t, const VectorX<T>& x,
                              const VectorX<T>& k) -> VectorX<T> {
      return VectorX<T>::Constant(1, scalar_ode_function(t, x[0], k));
    };
    // Instantiates the vector initial value problem.
    vector_ivp_ = std::make_unique<InitialValueProblem<T>>(
        ode_function, ToVectorIVPOdeContext(default_values));
  }

  /// Solves the IVP for time @p tf, using the initial time t₀, initial state
  /// x₀ and parameter vector 𝐤 present in @p values, falling back to the ones
  /// given on construction if not given.
  ///
  /// @param tf The IVP will be solved for this time.
  /// @param values IVP initial conditions and parameters.
  /// @returns The IVP solution x(@p tf; 𝐤) for x(t₀; 𝐤) = x₀.
  /// @pre Given @p tf must be larger than or equal to the specified initial
  ///      time t₀ (either given or default).
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector in the default specified
  ///      values given on construction.
  /// @throws std::exception if any of the preconditions is not met.
  T Solve(const T& tf, const ScalarOdeContext& values = {}) const {
    return this->vector_ivp_->Solve(tf, ToVectorIVPOdeContext(values))[0];
  }

  /// Solves and yields an approximation of the IVP solution x(t; 𝐤) for the
  /// closed time interval between the initial time t₀ and the given final
  /// time @p tf, using initial state x₀ and parameter vector 𝐤 present in
  /// @p values (falling back to the ones given on construction if not given).
  ///
  /// To this end, the wrapped IntegratorBase instance solves this scalar IVP,
  /// advancing time and state from t₀ and x₀ = x(t₀) to @p tf and x(@p tf),
  /// creating a scalar dense output over that [t₀, @p tf] interval along the
  /// way.
  ///
  /// @param tf The IVP will be solved up to this time. Usually, t₀ < @p tf as
  ///           an empty dense output would result if t₀ = @p tf.
  /// @param values IVP initial conditions and parameters.
  /// @returns A dense approximation to x(t; 𝐤) with x(t₀; 𝐤) = x₀, defined for
  ///          t₀ ≤ t ≤ tf.
  /// @note The larger the given @p tf value is, the larger the approximated
  ///       interval will be. See documentation of the specific dense output
  ///       technique in use for reference on performance impact as this
  ///       interval grows.
  /// @pre Given @p tf must be larger than or equal to the specified initial
  ///      time t₀ (either given or default).
  /// @pre If given, the dimension of the initial state vector @p values.x0
  ///      must match that of the default initial state vector in the default
  ///      specified values given on construction.
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector in the default specified
  ///      values given on construction.
  /// @throws std::exception if any of the preconditions is not met.
  std::unique_ptr<ScalarDenseOutput<T>> DenseSolve(
      const T& tf, const ScalarOdeContext& values = {}) const {
    // Delegates request to the vector form of this IVP by putting
    // specified values in vector form and the resulting dense output
    // back into scalar form.
    const int kDimension = 0;
    std::unique_ptr<DenseOutput<T>> vector_dense_output =
        this->vector_ivp_->DenseSolve(tf, ToVectorIVPOdeContext(values));
    return std::make_unique<ScalarViewDenseOutput<T>>(
        std::move(vector_dense_output), kDimension);
  }

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
  // Transforms given scalar IVP specified @p values into vector
  // IVP specified values.
  static typename InitialValueProblem<T>::OdeContext ToVectorIVPOdeContext(
      const ScalarOdeContext& values) {
    typename InitialValueProblem<T>::OdeContext vector_ivp_values;
    vector_ivp_values.k = values.k;
    vector_ivp_values.t0 = values.t0;
    if (values.x0.has_value()) {
      // Scalar initial state x₀ as a vector initial state 𝐱₀
      // of a single dimension.
      vector_ivp_values.x0 = VectorX<T>::Constant(1, values.x0.value()).eval();
    }
    return vector_ivp_values;
  }

  // Vector IVP representation of this scalar IVP.
  std::unique_ptr<InitialValueProblem<T>> vector_ivp_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ScalarInitialValueProblem)
