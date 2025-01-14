#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/scalar_dense_output.h"
#include "drake/systems/analysis/scalar_initial_value_problem.h"

namespace drake {
namespace systems {

/// A thin wrapper of the ScalarInitialValueProblem class that, in concert with
/// Drake's ODE initial value problem solvers ("integrators"), provide the
/// ability to perform quadrature on an arbitrary scalar integrable function.
/// That is, it allows the evaluation of an antiderivative function F(u; 𝐤),
/// such that F(u; 𝐤) = ∫ᵥᵘ f(x; 𝐤) dx where f : ℝ  →  ℝ , u ∈ ℝ, v ∈ ℝ,
/// 𝐤 ∈ ℝᵐ. The parameter vector 𝐤 allows for generic function definitions,
/// which can later be evaluated for any instance of said vector. Also, note
/// that 𝐤 can be understood as an m-tuple or as an element of ℝᵐ, the vector
/// space, depending on how it is used by the integrable function.
///
/// See ScalarInitialValueProblem class documentation for information
/// on caching support and dense output usage for improved efficiency in
/// antiderivative function F evaluation.
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
/// @tparam_nonsymbolic_scalar
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

  /// Constructs the antiderivative function of the given
  /// @p integrable_function, parameterized with @p k.
  ///
  /// @param integrable_function The function f(x; 𝐤) to be integrated.
  /// @param k 𝐤 ∈ ℝᵐ is the vector of parameters.  The default is the empty
  /// vector (indicating no parameters).
  AntiderivativeFunction(const IntegrableFunction& integrable_function,
                         const Eigen::Ref<const VectorX<T>>& k = Vector0<T>{});

  /// Evaluates the definite integral F(u; 𝐤) = ∫ᵥᵘ f(x; 𝐤) dx from the lower
  /// integration bound @p v to @p u using the parameter vector 𝐤 specified in
  /// the constructor (see definition in class documentation).
  ///
  /// @param v The lower integration bound.
  /// @param u The upper integration bound.
  /// @returns The value of the definite integral.
  /// @throws std::exception if v > u.
  T Evaluate(const T& v, const T& u) const;

  /// Evaluates and yields an approximation of the definite integral
  /// F(u; 𝐤) = ∫ᵥᵘ f(x; 𝐤) dx for v ≤ u ≤ w, i.e. the closed interval
  /// that goes from the lower integration bound @p v to the uppermost
  /// integration bound @p w, using the parameter vector 𝐤 specified in the
  /// constructor (see definition in class documentation).
  ///
  /// To this end, the wrapped IntegratorBase instance solves the integral
  /// from @p v to @p w (i.e. advances the state x of its differential form
  /// x'(t) = f(x; 𝐤) from @p v to @p w), creating a scalar dense output over
  /// that [@p v, @p w] interval along the way.
  ///
  /// @param v The lower integration bound.
  /// @param w The uppermost integration bound. Usually, @p v < @p w as an empty
  ///          dense output would result if @p v = @p w.
  /// @returns A dense approximation to F(u; 𝐤) (that is, a function), defined
  ///          for @p v ≤ u ≤ @p w.
  /// @note The larger the given @p w value is, the larger the approximated
  ///       interval will be. See documentation of the specific dense output
  ///       technique used by the internally held IntegratorBase subclass
  ///       instance for more details.
  /// @throws std::exception if v > w.
  std::unique_ptr<ScalarDenseOutput<T>> MakeDenseEvalFunction(const T& v,
                                                              const T& w) const;

  /// Resets the internal integrator instance.
  ///
  /// A usage example is shown below.
  /// @code{.cpp}
  ///    antiderivative_f.reset_integrator<RungeKutta2Integrator<T>>(max_step);
  /// @endcode
  ///
  /// @param args The integrator type-specific arguments.
  /// @returns The new integrator instance.
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

  /// Gets a reference to the internal integrator instance.
  const IntegratorBase<T>& get_integrator() const {
    return scalar_ivp_->get_integrator();
  }

  /// Gets a mutable reference to the internal integrator instance.
  IntegratorBase<T>& get_mutable_integrator() {
    return scalar_ivp_->get_mutable_integrator();
  }

 private:
  // Scalar IVP used to perform quadrature.
  std::unique_ptr<ScalarInitialValueProblem<T>> scalar_ivp_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::AntiderivativeFunction);
