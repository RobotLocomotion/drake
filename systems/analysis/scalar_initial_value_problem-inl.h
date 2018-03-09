#pragma once

#include <memory>
#include <utility>

#include "drake/systems/analysis/scalar_initial_value_problem.h"

namespace drake {
namespace systems {

template <typename T>
ScalarInitialValueProblem<T>::ScalarInitialValueProblem(
    const typename ScalarInitialValueProblem<T>::ScalarODEFunction&
    scalar_ode_function, const T& default_initial_time,
    const T& default_initial_state, const VectorX<T>& default_parameters) {
  // Wraps the given scalar ODE function as a vector ODE function.
  typename InitialValueProblem<T>::ODEFunction ode_function =
      [scalar_ode_function](const T& t, const VectorX<T>& x,
                            const VectorX<T>& k) -> VectorX<T> {
    return VectorX<T>::Constant(1, scalar_ode_function(t, x[0], k));
  };

  // Builds an initial state vector from the given scalar one.
  const VectorX<T> default_initial_state_as_vector =
      VectorX<T>::Constant(1, default_initial_state);

  // Instantiates the vector initial value problem.
  vector_ivp_ = std::make_unique<InitialValueProblem<T>>(
      ode_function, default_initial_time,
      default_initial_state_as_vector,
      default_parameters);
}

}  // namespace systems
}  // namespace drake
