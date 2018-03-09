#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/scalar_initial_value_problem.h"

namespace drake {
namespace systems {

/// A thin wrapper of the ScalarInitialValueProblem class to provide a simple
/// interface when performing quadrature on an arbitrary scalar function i.e.
/// when evaluating a primitive function F(u; 𝐤), such that
/// F(u; 𝐤) =∫ᵥᵘ f(x; 𝐤) dx where f : ℝ  →  ℝ , u ∈ ℝ, v ∈ ℝ, 𝐤 ∈ ℝᵐ. The
/// parameter vector 𝐤 allows for generic function definitions, which can later
/// be evaluated for any instance of said vector.
///
/// For further insight into its use, consider the following examples.
///
/// - Solving the elliptic integral of the first kind
///   F(φ; k) = ∫ᵠ √(1 - k² sin² θ)⁻¹ dθ becomes straightforward by defining
///   f(θ; 𝐤) ≜ √(1 - 𝐤₁² sin² θ)⁻¹ with 𝐤 ≜ [k] and evaluating at u = φ.
///
/// - As the bearings in a rotating machine age over time, these are more likely
///   to fail. Let γ be a random variable describing the time to first bearing
///   failure, described by a family of probability density functions fᵧ(y; l)
///   parameterized by bearing load l. In this context, the probability of a
///   bearing under a load l₁ to fail in the first N months becomes
///   P(0 < γ ≤ N mo.; l₁) = Fᵧ(N mo.; l₁) - Fᵧ(0; l₁), where
///   F'ᵧ(y; l) = fᵧ(y; l). Therefore, defining f ≜ fᵧ with 𝐤 ≜ [l] and
///   evaluating at u = N yields the result.
///
/// @tparam T The ℝ domain scalar type, which must be a valid Eigen scalar.
///
/// @note
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
template <typename T>
class PrimitiveFunction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrimitiveFunction);

  /// Scalar integrand function f(x; 𝐤) type.
  ///
  /// @param x The variable of integration x ∈ ℝ .
  /// @param k The integrand parameter vector 𝐤 ∈ ℝᵐ.
  /// @return The integrand value f(@p x; @p k).
  typedef std::function<T(const T& x, const VectorX<T>& k)> IntegrandFunction;

  /// A collection of values i.e. lower integration bound v
  /// and parameter vector 𝐤 to specify the integral.
  struct SpecifiedValues {
    /// Default constructor, leaving all values unspecified.
    SpecifiedValues() = default;

    /// Constructor specifying all values.
    ///
    /// @param v_in Specified lower integration bound v.
    /// @param k_in Specified parameter vector 𝐤.
    SpecifiedValues(const optional<T>& v_in,
                    const optional<VectorX<T>>& k_in)
        : v(v_in), k(k_in) {}

    optional<T> v;  ///< The lower integration bound v.
    optional<VectorX<T>> k;  ///< The parameter vector 𝐤.
  };

  /// Constructs the primitive function of the given @p integrand_function,
  /// using @p default_values.v as lower integration bound if given (0 if
  /// not) and parameterized with @p default_values.k by default.
  ///
  /// @param integrand_function The function f(x; 𝐤) under the integral sign.
  /// @param default_values The values specified by default for this function,
  ///                       i.e. default lower integration bound v ∈ ℝ  and
  ///                       default parameter vector 𝐤 ∈ ℝᵐ.
  /// @pre A parameter vector @p default_values.k is provided.
  /// @throw std::logic_error if preconditions are not met.
  PrimitiveFunction(const IntegrandFunction& integrand_function,
                    const SpecifiedValues& default_values) {
    // Expresses the scalar integral to be solved as an ODE.
    typename ScalarInitialValueProblem<T>::ScalarODEFunction
        scalar_ode_function = [integrand_function](
            const T& t, const T& x, const VectorX<T>& k) -> T {
      unused(x);
      return integrand_function(t, k);
    };

    // Default initial conditions for the scalar ODE form are
    // both, time and state, set to 0.
    typename ScalarInitialValueProblem<T>::SpecifiedValues
        scalar_ivp_default_values;
    scalar_ivp_default_values.t0 = default_values.v.value_or(
        static_cast<T>(0.0));
    scalar_ivp_default_values.x0 = static_cast<T>(0.0);
    scalar_ivp_default_values.k = default_values.k;

    // Instantiates the scalar initial value problem.
    scalar_ivp_ = std::make_unique<ScalarInitialValueProblem<T>>(
        scalar_ode_function, scalar_ivp_default_values);
  }

  /// Evaluates the function at @p u using the lower integration bound v and
  /// parameter vector 𝐤 present in @p values, falling back to the ones
  /// given on construction if not given.
  ///
  /// @param u The function argument (i.e. the upper integration bound).
  /// @param values The specified values for the function.
  /// @return The function value (i.e. the integration result).
  /// @pre Given upper integration bound @p u must be larger than or equal
  ///      to the lower integration bound v.
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector 𝐤 in the default specified
  ///      values given on construction.
  /// @throw std::logic_error if preconditions are not met.
  T Evaluate(const T& u, const SpecifiedValues& values = {}) const {
    typename ScalarInitialValueProblem<T>::SpecifiedValues
        scalar_ivp_values(values.v, {}, values.k);
    return scalar_ivp_->Solve(u, scalar_ivp_values);
  }

  /// Resets the internal integrator instance.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    primitive_f.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @return The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///                    IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          PrimitiveFunction::get_integrator() and
  ///          PrimitiveFunction::get_mutable_integrator().
  template <typename Integrator, typename... Args>
  Integrator* reset_integrator(Args&&... args) {
    return scalar_ivp_->template reset_integrator<Integrator>(
        std::forward<Args>(args)...);
  }

  /// Gets a pointer to the internal integrator instance.
  const IntegratorBase<T>* get_integrator() const {
    return scalar_ivp_->get_integrator();
  }

  /// Gets a pointer to the internal mutable integrator instance.
  IntegratorBase<T>* get_mutable_integrator() {
    return scalar_ivp_->get_mutable_integrator();
  }

 private:
  // Scalar IVP used to perform quadrature.
  std::unique_ptr<ScalarInitialValueProblem<T>> scalar_ivp_;
};

}  // namespace systems
}  // namespace drake
