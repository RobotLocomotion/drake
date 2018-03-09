#pragma once

#include <memory>
#include <utility>

#include "drake/common/unused.h"
#include "drake/systems/analysis/primitive_function.h"

namespace drake {
namespace systems {

template <typename T>
PrimitiveFunction<T>::PrimitiveFunction(
    const typename PrimitiveFunction<T>::IntegrandFunction& integrand_function,
    const VectorX<T>& default_parameters) {
  // Expresses the scalar integral to be solved as an ODE.
  typename ScalarInitialValueProblem<T>::ScalarODEFunction
      scalar_ode_function = [integrand_function](const T& t, const T& x,
                                                 const VectorX<T>& k) -> T {
    unused(x);
    return integrand_function(t, k);
  };

  // Default initial conditions for the scalar ODE are set to 0.
  const T default_initial_time = static_cast<T>(0.0);
  const T default_initial_state = static_cast<T>(0.0);

  // Instantiates the scalar initial value problem.
  scalar_ivp_ = std::make_unique<ScalarInitialValueProblem<T>>(
      scalar_ode_function, default_initial_time,
      default_initial_state, default_parameters);
}

}  // namespace systems
}  // namespace drake
