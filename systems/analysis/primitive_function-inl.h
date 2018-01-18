#pragma once

#include <memory>
#include <utility>

#include "drake/systems/analysis/primitive_function.h"

namespace drake {
namespace systems {

template <typename T>
PrimitiveFunction<T>::PrimitiveFunction(
    const typename PrimitiveFunction<T>::IntegrandFunction& integrand_function,
    const Parameters<T>& default_parameters) {
  // Generalizes the given scalar integrand function to build a system.
  typename ScalarInitialValueProblem<T>::ScalarODEFunction
      scalar_ode_function = [integrand_function](const T& t, const T& x,
                                                 const Parameters<T>& k) -> T {
    unused(x);
    return integrand_function(t, k);
  };

  // Instantiates a single element state vector model using the given constant.
  const T default_initial_time = static_cast<T>(0.0);
  const T default_initial_state = static_cast<T>(0.0);
  // Instantiates the ivp solver.
  scalar_ivp_ = std::make_unique<ScalarInitialValueProblem<T>>(
      scalar_ode_function, default_initial_time,
      default_initial_state, default_parameters);
}

}  // namespace systems
}  // namespace drake
