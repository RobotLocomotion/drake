#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/scalar_initial_value_problem.h"

namespace drake {
namespace systems {

/// A primitive function F(x; 𝐤) representation class, such that
/// F'(x; 𝐤) = f(x; 𝐤) where f : ℝ  →  ℝ , x ∈ ℝ, 𝐤 ∈ ℝᵐ. In short, this
/// abstraction allows to perform quadrature on an arbitrary scalar function.
/// Lower and upper integration bounds can be set independently.
///
/// For further insight into its use, consider the following examples.
///
/// - Solving the elliptic integral of the first kind
///   F(φ; k) = ∫ᵠ √(1 - k² sin² θ)⁻¹ dθ becomes straightforward by defining
///   f(θ; 𝐤) ≜ √(1 - 𝐤₁² sin² θ)⁻¹, 𝐤 ≜ [k] and integrating from 0 to φ.
///
/// - As the bearings in a rotating machine age over time, these are more likely
///   to fail. Let γ be a random variable describing the time to first bearing
///   failure, described by a family of probability density functions fᵧ(x; l)
///   parameterized by bearing load l. In this context, the probability of a
///   bearing under a load l₁ to fail in the first N months becomes
///   P(0 < y ≤ N mo.; l₁) = Fᵧ(N mo.; l₁) - Fᵧ(0; l₁), where
///   F'ᵧ(x; l) = fᵧ(x; l). Therefore, f ≜ fᵧ, 𝐤 ≜ [l], and integrating from 0
///   to N yields the result.
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

  /// Scalar integrand function F(x; 𝐤) type.
  ///
  /// @param x The variable of integration x ∈ ℝ .
  /// @param k The integrand parameters vector 𝐤 ∈ ℝᵐ.
  /// @return The integrand value f(@p x ; @p k ).
  typedef std::function<T(const T& x, const VectorX<T>& k)> IntegrandFunction;

  /// Constructs the primitive function of the given @p integrand_function,
  /// parameterized with @p default_parameters by default.
  ///
  /// @param integrand_function The function F(x; 𝐤) under the integral sign.
  /// @param default_parameters The default parameters vector 𝐤₀ ∈ ℝᵐ for the
  /// @p integrand_function.
  PrimitiveFunction(const IntegrandFunction& integrand_function,
                    const VectorX<T>& default_parameters);

  /// Evaluates the function by integrating from 0 to @p b using default
  /// parameters 𝐤₀.
  /// @param b The upper integration bound.
  /// @return The integration result.
  /// @pre The upper integration bound @p b is non-negative.
  /// @warning This method will abort if preconditions are not met.
  inline T Evaluate(const T& b) const {
    return scalar_ivp_->Solve(b);
  }

  /// Evaluates the function by integrating from 0 to @p b using given
  /// parameters @p k.
  /// @param b The upper integration bound.
  /// @param k The integrand parameters vector.
  /// @return The integration result.
  /// @pre The upper integration bound @p b is non-negative.
  /// @pre The dimensions of the given @p k parameters must match that of
  ///      the default parameters vector 𝐤₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T Evaluate(const T& b, const VectorX<T>& k) const {
    return scalar_ivp_->Solve(b, k);
  }

  /// Evaluates the function by integrating from @p a to @p b using the default
  /// parameters vector 𝐤₀.
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @return The integration result.
  /// @pre The upper integration bound @p b is larger than the lower integration
  /// bound @p a.
  /// @warning This method will abort if preconditions are not met.
  inline T Evaluate(const T& a, const T& b) const {
    return scalar_ivp_->Solve(a, b);
  }

  /// Evaluates the function by integrating from @p a to @p b using the given
  /// parameters vector @p k.
  ///
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @param k The integrand parameters vector.
  /// @return The integration result.
  /// @pre The upper integration bound @p b is larger than the lower
  ///      integration bound @p a.
  /// @pre The dimension of the given parameters vector @p k must match that
  ///      of the default parameters vector 𝐤₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  T Evaluate(const T& a, const T& b, const VectorX<T>& k) const {
    return scalar_ivp_->Solve(a, b, k);
  }

  /// Resets the internal integrator instance.
  /// @return The new integrator instance.
  /// @tparam I The integrator type, which must be an IntegratorBase subclass.
  template <typename I>
  inline IntegratorBase<T>* reset_integrator() {
    return scalar_ivp_->template reset_integrator<I>();
  }

  inline const IntegratorBase<T>* get_integrator() const {
    return scalar_ivp_->get_integrator();
  }

  inline IntegratorBase<T>* get_mutable_integrator() {
    return scalar_ivp_->get_mutable_integrator();
  }

 private:
  // Scalar IVP used for quadrature.
  std::unique_ptr<ScalarInitialValueProblem<T>> scalar_ivp_;
};

}  // namespace systems
}  // namespace drake
