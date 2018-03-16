#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/scalar_initial_value_problem.h"

namespace drake {
namespace systems {

/// A thin wrapper of the ScalarInitialValueProblem class that, in concert with
/// Drake's ODE initial value problem solvers ("integrators"), provide the
/// ability to perform quadrature on an arbitrary scalar integrable function.
/// That is, it allows the evaluation of an antiderivative function F(u; 𝐤),
/// such that F(u; 𝐤) =∫ᵥᵘ f(x; 𝐤) dx where f : ℝ  →  ℝ , u ∈ ℝ, v ∈ ℝ,
/// 𝐤 ∈ ℝᵐ. The parameter vector 𝐤 allows for generic function definitions,
/// which can later be evaluated for any instance of said vector. Also, note
/// that 𝐤 can be understood as an m-tuple or as an element of ℝᵐ, the vector
/// space, depending on how it is used by the integrable function.
///
/// For further insight into its use, consider the following examples.
///
/// - Solving the elliptic integral of the first kind
///   E(φ; ξ) = ∫ᵠ √(1 - ξ² sin² θ)⁻¹ dθ becomes straightforward by defining
///   f(x; 𝐤) ≜ √(1 - k₀² sin² x)⁻¹ with 𝐤 ≜ [ξ] and evaluating F(u; 𝐤) at
///   u = φ.
///
/// - As the bearings in a rotating machine age over time, these are more likely
///   to fail. Let γ be a random variable describing the time to first bearing
///   failure, described by a family of probability density functions gᵧ(y; l)
///   parameterized by bearing load l. In this context, the probability of a
///   bearing under load to fail during the first N months becomes
///   P(0 < γ ≤ N mo.; l) = Gᵧ(N mo.; l) - Gᵧ(0; l), where Gᵧ(y; l) is the
///   family of cumulative density functions, parameterized by bearing load l,
///   and G'ᵧ(y; l) = gᵧ(y; l). Therefore, defining f(x; 𝐤) ≜ gᵧ(x; k₀) with
///   𝐤 ≜ [l] and evaluating F(u; 𝐤) at u = N yields the result.
///
/// @tparam T The ℝ domain scalar type, which must be a valid Eigen scalar.
///
/// @note
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
template <typename T>
class AntiderivativeFunction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AntiderivativeFunction);

  /// Scalar integrable function f(x; 𝐤) type.
  ///
  /// @param x The variable of integration x ∈ ℝ .
  /// @param k The parameter vector 𝐤 ∈ ℝᵐ.
  /// @return The function value f(@p x; @p k).
  using IntegrableFunction = std::function<T(const T& x, const VectorX<T>& k)>;

  /// Approximation technique function type, to build an approximating function
  /// G(u) to an F(u; 𝐤) =∫ᵥᵘ f(x; 𝐤) dx antiderivative function based on a
  /// partition of its domain for which function value and first derivative are
  /// known and provided at multiple values of u.
  ///
  /// @param u_sequence The integration upper bound sequence
  ///                   (u₁ ... uₚ) where uₚ ∈ ℝ.
  /// @param f_sequence The dependent scalar variable sequence
  ///                   (F(u₁; 𝐤) ... F(uₚ; 𝐤)) where F(uₚ; 𝐤) ∈ ℝ.
  /// @param dfdt_sequence The dependent scalar variable first derivative
  ///                      sequence (f(u₁; 𝐤) ... f(uₚ; 𝐤)) where
  ///                      f(uₚ; 𝐤) ∈ ℝ.
  /// @return The approximating function G(u) ∈ ℝ.
  /// @tparam ApproximatingFn The approximating function type.
  template <typename ApproximatingFn>
  using ApproximationTechnique = std::function<ApproximatingFn (
      const std::vector<T>& u_sequence,
      const std::vector<T>& f_sequence,
      const std::vector<T>& dfdt_sequence)>;

  /// The set of values that, along with the function being integrated,
  /// partially specify the definite integral i.e. providing the lower
  /// integration bound v and the parameter vector 𝐤, leaving the upper
  /// integration bound u to be specified on evaluation.
  struct SpecifiedValues {
    /// Default constructor that leaves all values unspecified.
    SpecifiedValues() = default;

    /// Constructor that specifies all values.
    /// @param v_in Specified lower integration bound v.
    /// @param k_in Specified parameter vector 𝐤.
    SpecifiedValues(const optional<T>& v_in,
                    const optional<VectorX<T>>& k_in)
        : v(v_in), k(k_in) {}

    optional<T> v;  ///< The lower integration bound v.
    optional<VectorX<T>> k;  ///< The parameter vector 𝐤.
  };

  /// Constructs the antiderivative function of the given
  /// @p integrable_function, using @p default_values.v as lower integration
  /// bound if given (0 if not) and parameterized with @p default_values.k if
  /// given (an empty vector if not) by default.
  ///
  /// @param integrable_function The function f(x; 𝐤) to be integrated.
  /// @param default_values The values specified by default for this function,
  ///                       i.e. default lower integration bound v ∈ ℝ  and
  ///                       default parameter vector 𝐤 ∈ ℝᵐ.
  AntiderivativeFunction(const IntegrableFunction& integrable_function,
                         const SpecifiedValues& default_values = {}) {
    // Expresses the scalar integral to be solved as an ODE.
    typename ScalarInitialValueProblem<T>::ScalarODEFunction
        scalar_ode_function = [integrable_function](
            const T& t, const T& x, const VectorX<T>& k) -> T {
      unused(x);
      return integrable_function(t, k);
    };

    typename ScalarInitialValueProblem<T>::SpecifiedValues
        scalar_ivp_default_values;
    // Default initial time for the scalar ODE form falls back
    // to 0 if no lower integration bound is specified.
    scalar_ivp_default_values.t0 = default_values.v.value_or(
        static_cast<T>(0.0));
    // Default initial state for the scalar ODE form is set to 0.
    scalar_ivp_default_values.x0 = static_cast<T>(0.0);
    // Default parameter vector for the scalar ODE falls back to
    // the empty vector if none is given.
    scalar_ivp_default_values.k = default_values.k.value_or(VectorX<T>());

    // Instantiates the scalar initial value problem.
    scalar_ivp_ = std::make_unique<ScalarInitialValueProblem<T>>(
        scalar_ode_function, scalar_ivp_default_values);
  }

  /// Evaluates the definite integral F(u; 𝐤) =∫ᵥᵘ f(x; 𝐤) dx from the lower
  /// integration bound v (see definition in class documentation) to @p u using
  /// the parameter vector 𝐤 (see definition in class documentation) if present
  /// in @p values, falling back to the ones given on construction if missing.
  ///
  /// @param u The upper integration bound.
  /// @param values The specified values for the integration.
  /// @return The value of the definite integral.
  /// @pre The given upper integration bound @p u must be larger than or equal
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

  /// Approximates the definite integral F(u; 𝐤) =∫ᵥᵘ f(x; 𝐤) dx with
  /// another function G(w) defined for v <= w <= @p u using the given
  /// @p approximation_technique. Above's lower integration bound v and
  /// parameter vector 𝐤 (see definition in class documentation) are taken from
  /// @p values, falling back to the ones given on construction if missing.
  ///
  /// @param approximation_technique The technique to build the approximating
  ///                                function G(w).
  /// @param u The uppermost integration bound.
  /// @param values The specified values for the integration.
  /// @return The approximating function G(w) to the definite integral for
  ///         for v <= w <= @p u.
  /// @tparam ApproximatingFn The approximating function type.
  /// @pre The given upper integration bound @p u must be larger than or equal
  ///      to the lower integration bound v.
  /// @pre If given, the dimension of the parameter vector @p values.k
  ///      must match that of the parameter vector 𝐤 in the default specified
  ///      values given on construction.
  /// @throw std::logic_error if preconditions are not met.
  /// @note See ScalarInitialValueProblem::Approximate() documentation for
  ///       further reference.
  template <typename ApproximatingFn>
  ApproximatingFn Approximate(
      const ApproximationTechnique<ApproximatingFn>& approximation_technique,
      const T& u, const SpecifiedValues& values = {}) const {
    typename ScalarInitialValueProblem<T>::SpecifiedValues
        scalar_ivp_values(values.v, {}, values.k);
    return this->scalar_ivp_->Approximate(
        approximation_technique, u, scalar_ivp_values);
  }

  /// Resets the internal integrator instance.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    antiderivative_f.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @return The new integrator instance.
  /// @tparam Integrator The integrator type, which must be an
  ///                    IntegratorBase subclass.
  /// @tparam Args The integrator specific argument types.
  /// @warning This operation invalidates pointers returned by
  ///          AntiderivativeFunction::get_integrator() and
  ///          AntiderivativeFunction::get_mutable_integrator().
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
