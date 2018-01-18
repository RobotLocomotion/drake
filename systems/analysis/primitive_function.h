#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/scalar_initial_value_problem.h"
#include "drake/systems/framework/parameters.h"

namespace drake {
namespace systems {

/// A primitive function F(x; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) representation class, such that
/// F'(x; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) = f(x; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) where f : ℝ  →  ℝ ,
/// 𝐤₀ ∈ ℝ ᵐ⁰, 𝐤₁ ∈ ℝ ᵐ¹, ..., 𝐤ₙ ∈ ℝ ᵐⁿ. In short, this abstraction
/// allows to perform quadrature on an arbitrary scalar function. Lower and
/// upper integration bounds can be set independently.
///
/// For further insight into its use, consider the following examples.
///
/// - Solving the elliptic integral of the first kind
///   F(φ; k) = ∫ᵠ √(1 - k² sin² θ)⁻¹ dθ becomes straightforward by defining
///   f(θ; k) ≜ √(1 - k² sin² θ)⁻¹, 𝐤₀ ≜ [k] and integrating from 0 to φ.
///
/// - As the bearings in a rotating machine age over time, these are more likely
///   to fail. Let γ be a random variable describing the time to first bearing
///   failure, described by a family of probability density functions fᵧ(x; l)
///   parameterized by bearing load l. In this context, the probability of a
///   bearing under a load l₁ to fail in the first N months becomes
///   P(0 < y ≤ N mo.; l₁) = Fᵧ(N mo.; l₁) - Fᵧ(0; l₁), where
///   F'ᵧ(x; l) = fᵧ(x; l). Therefore, f ≜ fᵧ, 𝐤₀ ≜ [l], and integrating from 0
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

  /// Scalar integrand function F(x; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) type.
  ///
  /// @param x The variable of integration x ∈ ℝ .
  /// @param k The integrand parameters 𝐤₀ ∈ ℝ ᵐ⁰, 𝐤₁ ∈ ℝ ᵐ¹, ..., 𝐤ₙ ∈ ℝ ᵐⁿ.
  /// @return The integrand value f(@p x ; @p k ).
  typedef std::function<T(const T& x,
                          const Parameters<T>& k)> IntegrandFunction;

  /// Constructs the primitive function of the given @p integrand_function,
  /// parameterized with @p default_parameters by default.
  ///
  /// @param integrand_function The function F(x; 𝐤₀, 𝐤₁, ..., 𝐤ₙ) under the
  /// integral sign.
  /// @param default_parameters The default parameters 𝐤₀ ∈ ℝ ᵐ⁰, 𝐤₁ ∈ ℝ ᵐ¹,
  /// ..., 𝐤ₙ ∈ ℝ ᵐⁿ. for the @p integrand_function.
  PrimitiveFunction(const IntegrandFunction& integrand_function,
                    const Parameters<T>& default_parameters);

  /// Evaluates the function by integrating from 0 to @p b using default
  /// parameters.
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
  /// @param k The integrand parameters 𝐤₀, 𝐤₁, ..., 𝐤ₙ.
  /// @return The integration result.
  /// @pre The upper integration bound @p b is non-negative.
  ///  @pre The dimensions of the given @p p parameters must match that of
  /// the default parameters vector given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T Evaluate(const T& b, const Parameters<T>& p) const {
    return scalar_ivp_->Solve(b, p);
  }

  /// Evaluates the function by integrating from @p a to @p b using default
  /// parameters.
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @return The integration result.
  /// @pre The upper integration bound @p b is larger than the lower integration
  /// bound @p a.
  /// @warning This method will abort if preconditions are not met.
  inline T Evaluate(const T& a, const T& b) const {
    return scalar_ivp_->Solve(a, b);
  }

  /// Evaluates the function by integrating from @p a to @p b using given
  /// parameters @p k.
  ///
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @param k The integrand parameters 𝐤₀, 𝐤₁, ..., 𝐤ₙ.
  /// @return The integration result.
  /// @pre The upper integration bound @p b is larger than the lower
  /// integration bound @p a.
  /// @pre The quantity and dimension of the given parameters @p p must match
  /// that of the default parameters 𝐩₀ given on construction.
  /// @warning This method will abort if preconditions are not met.
  T Evaluate(const T& a, const T& b, const Parameters<T>& k) const {
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
