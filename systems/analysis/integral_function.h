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

/// A parameterizable integral function, that explicitly solves
/// dy/dx = F(x, y, 𝐩) ODE, where F : 𝕊 → 𝕊 , x ∈ 𝕊 , y ∈ 𝕊, 𝐩 ∈ 𝕊ⁱ.
///
/// @tparam T The 𝕊 domain scalar type, which must be a valid Eigen scalar.
///
/// @note
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
template <typename T>
class IntegralFunction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntegralFunction);

  /// Scalar integrand function type .
  ///
  /// @param x The variable of integration.
  /// @param y The integral result up to @p x.
  /// @param p The vector of parameters.
  /// @return The integrand at @p x, @p y, parameterized with @p p.
  typedef std::function<T(const T& x, const T& y, const VectorX<T>& p)>
      IntegrandFunction;

  /// Constructs a parameterizable scalar integral function.
  ///
  /// @param integrand_function The scalar function under the integral sign.
  /// @param constant_of_integration An additive constant C.
  /// @param default_parameters The default parameters for the integrand.
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
  /// @return The integration result.
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
  inline T operator()(const T& a, const T& b) const {
    return this->operator()(a, b, default_parameters_);
  }

  /// Evaluates the function, integrating from @p a to @p b and parameterizing
  /// with @p p.
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @param p The vector of parameters.
  /// @return The integration result.
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
  // Checks wheter a given systems::Context can be used to integrate from the
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
