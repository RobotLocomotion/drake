#pragma once

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
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
///
/// Note the distinction from general initial value problems where
/// f : t ⨯ 𝐱 → ℝⁿ and 𝐱 ∈ ℝⁿ, addressed by the class being wrapped. While
/// every scalar initial value problem could be written in vector form, this
/// wrapper keeps both problem definition and solution in their scalar form
/// with almost zero overhead, leading to clearer code if applicable.
/// Moreover, this scalar form facilitates single-dimensional quadrature
/// using methods for solving initial value problems.
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
  using ScalarODEFunction = std::function<T(const T& t, const T& x,
                                            const VectorX<T>& k)>;

  /// Approximation technique function type, to build an approximating function
  /// z(t) to an x(t; 𝐤) solution based on a partition of its domain for which
  /// value x and first derivative dx/dt are known and provided at multiple
  /// values of t.
  ///
  /// @param t_sequence The independent scalar variable sequence
  ///                   (t₁ ... tₚ) where tₚ ∈ ℝ.
  /// @param x_sequence The dependent scalar variable sequence
  ///                   (x(t₁) ... x(tₚ)) where x(tₚ) ∈ ℝ.
  /// @param dx_dt_sequence The dependent scalar variable first derivative
  ///                       sequence ((dx/dt)(t₁) ... (dx/dt)(tₚ)) where
  ///                       (dx/dt)(tₚ) ∈ ℝ.
  /// @return The approximating function z(t) ∈ ℝ.
  /// @tparam ApproximatingFn The approximating function type.
  template <typename ApproximatingFn>
  using ApproximationTechnique = std::function<ApproximatingFn (
      const std::vector<T>& t, const std::vector<T>& x,
      const std::vector<T>& dx_dt)>;

  /// A collection of values i.e. initial time t₀, initial state x₀
  /// and parameter vector 𝐤 to further specify the ODE system (in
  /// order to become a scalar initial value problem).
  struct SpecifiedValues {
    /// Default constructor, leaving all values unspecified.
    SpecifiedValues() = default;

    /// Constructor specifying all values.
    ///
    /// @param t0_in Specified initial time t₀.
    /// @param x0_in Specified initial state x₀.
    /// @param k_in Specified parameter vector 𝐤.
    SpecifiedValues(const optional<T>& t0_in,
                    const optional<T>& x0_in,
                    const optional<VectorX<T>>& k_in)
        : t0(t0_in), x0(x0_in), k(k_in) {}

    optional<T> t0;  ///< The initial time t₀ for the IVP.
    optional<T> x0;  ///< The initial state x₀ for the IVP.
    optional<VectorX<T>> k;  ///< The parameter vector 𝐤 for the IVP.
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
  /// @throw std::logic_error if preconditions are not met.
  ScalarInitialValueProblem(const ScalarODEFunction& scalar_ode_function,
                            const SpecifiedValues& default_values) {
    // Wraps the given scalar ODE function as a vector ODE function.
    typename InitialValueProblem<T>::ODEFunction ode_function =
        [scalar_ode_function](const T& t, const VectorX<T>& x,
                              const VectorX<T>& k) -> VectorX<T> {
      return VectorX<T>::Constant(1, scalar_ode_function(t, x[0], k));
    };
    // Instantiates the vector initial value problem.
    vector_ivp_ = std::make_unique<InitialValueProblem<T>>(
        ode_function, ToVectorIVPSpecifiedValues(default_values));
  }

  /// Solves the IVP for time @p tf, using the initial time t₀, initial state
  /// x₀ and parameter vector 𝐤 present in @p values, falling back to the ones
  /// given on construction if not given.
  ///
  /// @param tf The time to solve the IVP for.
  /// @param values The specified values for the IVP.
  /// @return The IVP solution x(@p tf; 𝐤) for x(t₀; 𝐤) = x₀.
  /// @pre Given @p tf must be larger than or equal to the specified initial
  ///      time t₀ (either given or default).
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector in the default specified
  ///      values given on construction.
  /// @throw std::logic_error if preconditions are not met.
  T Solve(const T& tf, const SpecifiedValues& values = {}) const {
    return this->vector_ivp_->Solve(tf, ToVectorIVPSpecifiedValues(values))[0];
  }

  /// Approximates the IVP solution x(t; 𝐤) with x(t₀; 𝐤) = x₀ using
  /// another function z(t) defined for t₀ <= t <= @p tf using the
  /// given @p approximation_technique.
  ///
  /// @param approximation_technique The technique to build the approximating
  ///                                function z(t).
  /// @param tf The time to solve the IVP for.
  /// @param values The specified values for the IVP.
  /// @return The approximating function z(t) to the IVP solution x(t; 𝐤)
  ///         with x(t₀; 𝐤) = x₀ for t₀ <= t <= @p tf.
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
  /// @note See InitialValueProblem::Approximate() documentation for
  ///       further reference.
  template <typename ApproximatingFn>
  ApproximatingFn Approximate(
      const ApproximationTechnique<ApproximatingFn>& approximation_technique,
      const T& tf, const SpecifiedValues& values = {}) const {
    return this->vector_ivp_->Approximate(
        ToVectorApproximationTechnique(approximation_technique),
        tf, ToVectorIVPSpecifiedValues(values));
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
  /// @return The new integrator instance.
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

  /// Gets a pointer to the internal integrator instance.
  const IntegratorBase<T>* get_integrator() const {
    return vector_ivp_->get_integrator();
  }

  /// Gets a pointer to the internal mutable integrator instance.
  IntegratorBase<T>* get_mutable_integrator() {
    return vector_ivp_->get_mutable_integrator();
  }

 private:
  // Transforms given scalar IVP specified @p values into vector
  // IVP specified values.
  static typename InitialValueProblem<T>::SpecifiedValues
  ToVectorIVPSpecifiedValues(const SpecifiedValues& values) {
    typename InitialValueProblem<T>::SpecifiedValues vector_ivp_values;
    vector_ivp_values.k = values.k;
    vector_ivp_values.t0 = values.t0;
    if (values.x0.has_value()) {
      vector_ivp_values.x0 = VectorX<T>::Constant(
          1, values.x0.value()).eval();
    }
    return vector_ivp_values;
  }

  template <typename A>
  using VectorApproximationTechnique =
      typename InitialValueProblem<T>::template ApproximationTechnique<A>;

  // Transforms given scalar @p approximation_technique into its vector form.
  template <typename ApproximatingFn>
  static VectorApproximationTechnique<ApproximatingFn>
  ToVectorApproximationTechnique(
      const ApproximationTechnique<ApproximatingFn>& approximation_technique) {
    return [&approximation_technique](
        const std::vector<T>& t_sequence,
        const std::vector<VectorX<T>>& x_sequence,
        const std::vector<VectorX<T>>& dxdt_sequence) {
      auto vector_to_scalar = [](const VectorX<T>& v) { return v[0]; };
      std::vector<T> scalar_x_sequence(x_sequence.size());
      std::transform(x_sequence.begin(), x_sequence.end(),
                     scalar_x_sequence.begin(), vector_to_scalar);
      std::vector<T> scalar_dxdt_sequence(x_sequence.size());
      std::transform(dxdt_sequence.begin(), dxdt_sequence.end(),
                     scalar_dxdt_sequence.begin(), vector_to_scalar);
      return approximation_technique(
          t_sequence, scalar_x_sequence, scalar_dxdt_sequence);
    };
  }

  // Vector IVP representation of this scalar IVP.
  std::unique_ptr<InitialValueProblem<T>> vector_ivp_;
};

}  // namespace systems
}  // namespace drake
