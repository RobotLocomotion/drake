#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace analysis {

/// A parameterizable primitive function y = F(x) that explicitly solves
/// F'(x) = dy/dx = f(x, y, 𝐩) with F(0) = C, where
/// f : x ⨯ y ⊆ ℝ ² →  dy/dx ⊆ ℝ , x ∈ ℝ ₀⁺, y ∈ ℝ , 𝐩 ∈ ℝ ⁱ and C ∈ ℝ .
///
/// For further insight on its use, consider the following example. The charge
/// Q stored in the capacitor of a series RC circuit (resistor Rs, capacitor Cs)
/// driven by a time varying voltage source E(t).can be described by
/// dQ/dt = (E(t) - Q / Cs) / Rs. This adds up to the initally stored charge Q₀.
/// In this context, x ≜ t, y ≜ Q, 𝐩 ≜ [Rs, Cs], C ≜ Q₀, dy/dx = f(x, y, 𝐩) =
/// (E(x) - y / p₂) / p₁.
///
/// @tparam T The ℝ domain scalar type, which must be a valid Eigen scalar.
///
/// @note
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
template <typename T>
class IntegralFunction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntegralFunction);

  /// Default integration accuracy in the relative tolerance sense.
  static const T kDefaultAccuracy;
  /// Default initial integration step size as it's adapted to meet
  /// accuracy requirements.
  static const T kInitialStepSize;
  /// Default maximum integration step size as it's adapted to meet
  /// accuracy requirements.
  static const T kMaxStepSize;

  /// Scalar integrand function `f` type.
  ///
  /// @param x The variable of integration x ∈ ℝ ₀⁺.
  /// @param y The integral result y ∈ ℝ up to @p x.
  /// @param p The vector of parameters 𝐩 ∈ ℝ ⁱ.
  /// @return The integrand value at (@p x, @p y), parameterized with @p p.
  typedef std::function<T(const T& x, const T& y, const VectorX<T>& p)>
      IntegrandFunction;

  /// Constructs a parameterizable scalar integral function.
  ///
  /// @param integrand_function The scalar integrand function `f` under
  /// the integral sign.
  /// @param constant_of_integration The additive constant F(0) = C.
  /// @param default_parameters The default parameters for the integrand
  /// function.
  IntegralFunction(const IntegrandFunction& integrand_function,
                   const T& constant_of_integration,
                   const VectorX<T>& default_parameters);

  /// Evaluates the function, integrating from 0 to @p b using default
  /// parameters.
  /// @param b The upper integration bound.
  /// @return The integration result.
  inline T operator()(const T& b) const {
    return this->operator()(b, default_parameters_);
  }

  /// Evaluates the function, integrating from 0 to @p b and parameterizing
  /// with @p p.
  /// @param b The upper integration bound.
  /// @param p The vector of parameters.
  /// @return The integration result..
  /// @pre The upper integration bound @p b is a non-negative real number.
  /// @pre The size of the given @p params vector must match that of the default
  /// parameters vector given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T operator()(const T& b, const VectorX<T>& p) const {
    return this->operator()(static_cast<T>(0.), b, p);
  }

  /// Evaluates the function, integrating from @p a to @p b using default
  /// parameters.
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @return The integration result.
  /// @pre The lower integration bound @p a is a non-negative real number .
  /// @pre The upper integration bound @p b is larger than the lower integration
  /// bound.
  /// @warning This method will abort if preconditions are not met.
  inline T operator()(const T& a, const T& b) const {
    return this->operator()(a, b, default_parameters_);
  }

  /// Evaluates the function, integrating from @p a to @p b and parameterizing
  /// with @p p.
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @param p The vector of parameters.
  /// @return The integration result.
  /// @pre The lower integration bound @p a is a non-negative real number .
  /// @pre The upper integration bound @p b is larger than the lower integration
  /// bound.
  /// @pre The size of the given @p params vector must match that of the
  /// default parameters vector given on construction.
  /// @warning This method will abort if preconditions are not met.
  T operator()(const T& a, const T& b, const VectorX<T>& p) const;

  /// Resets the internal integrator instance.
  /// @return The new integrator instance.
  /// @tparam I The integrator type, which must be an IntegratorBase subclass.
  template <typename I>
  IntegratorBase<T>* reset_integrator();

  inline const IntegratorBase<T>* get_integrator() const {
    return integrator_.get();
  }

  inline IntegratorBase<T>* get_mutable_integrator() {
    return integrator_.get();
  }

 private:
  // Checks whether a given systems::Context can be used to integrate from the
  // given @p lower_integration_bound up to the given @p upper_integration_bound
  // and given @p parameters, allowing to optimize away integration context
  // setups for successive, incremental evaluations of the function with the
  // same parameterization.
  //
  // @param context Current integration context to be checked.
  // @param lower_integration_bound The new lower integration bound.
  // @param upper_integration_bound The new upper integration bound.
  // @param parameters The new vector of parameters.
  // @return True if the same context can be used to perform the
  // integration, False otherwise.
  bool IsContextValid(const Context<T>& context,
                      const T& lower_integration_bound,
                      const T& upper_integration_bound,
                      const VectorX<T>& parameters) const;

  // ODE system integration context cache.
  mutable std::unique_ptr<Context<T>> context_;
  // ODE system integration initial time cache.
  mutable T initial_context_time_;
  // ODE system representation used for function evaluation.
  std::unique_ptr<System<T>> system_;
  // ODE system representation default parameters.
  const VectorX<T> default_parameters_;
  // Numerical integrator used for function evaluation.
  std::unique_ptr<IntegratorBase<T>> integrator_;
};

}  // namespace analysis
}  // namespace systems
}  // namespace drake
