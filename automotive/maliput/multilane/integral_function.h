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
namespace maliput {
namespace multilane {

// A parameterizable integral function.
//
// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class IntegralFunction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntegralFunction);

  /// Scalar integrand function type.
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
  /// @param parameters The default parameters for the integrand.
  IntegralFunction(const IntegrandFunction& integrand_function,
                   const T& constant_of_integration,
                   const VectorX<T>& parameters);

  /// Evaluates the function, integrating from 0 to @p b and
  /// parameterizing with @p p.
  ///
  /// @param b The upper integration bound.
  /// @param p The vector of parameters.
  /// @return The integration result.
  /// @pre The size of the given @p params vector must match that of vector
  /// given on construction.
  /// @warning This method will abort if preconditions are not met.
  T operator()(const T& b, const VectorX<T>& p) const;

  /// Evaluates the function, integrating from @p a to @p b and
  /// parameterizing with @p p.
  ///
  /// @param a The lower integration bound.
  /// @param b The upper integration bound.
  /// @param p The vector of parameters.
  /// @return The integration result.
  /// @pre The size of the given @p params vector must match that of vector
  /// given on construction.
  /// @warning This method will abort if preconditions are not met.
  inline T operator()(const T& a, const T& b, const VectorX<T>& p) const {
    return -(this->operator()(a, p) - this->operator()(b, p));
  }

  /// Resets the internal integrator instance.
  /// @return The new integrator instance.
  /// @tparam I The integrator type, which must be an IntegratorBase subclass.
  template <typename I>
  systems::IntegratorBase<T>* reset_integrator();

  inline const systems::IntegratorBase<T>* get_integrator() const {
    return integrator_.get();
  }

  inline systems::IntegratorBase<T>* get_mutable_integrator() {
    return integrator_.get();
  }

 private:
  // Checks wheter a given systems::Context can be used to integrate up to the
  // given @p upper_integration_bound and given @p parameters, allowing to
  // optimize away integration context setups for successive, incremental
  // evaluations of the function with the same parameterization.
  //
  // @param context Current integration context to be checked.
  // @param upper_integration_bound The new upper integration bound.
  // @param parameters The new vector of parameters.
  // @return True if the same context can be used to perform the
  // integration, False otherwise.
  bool IsContextValid(const systems::Context<T>& context,
                      const T& upper_integration_bound,
                      const VectorX<T>& parameters) const;

  // ODE system integration context cache.
  mutable std::unique_ptr<systems::Context<T>> context_;
  // ODE system representation used for function evaluation.
  std::unique_ptr<systems::System<T>> system_;
  // Numerical integrator used for function evaluation.
  std::unique_ptr<systems::IntegratorBase<T>> integrator_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
