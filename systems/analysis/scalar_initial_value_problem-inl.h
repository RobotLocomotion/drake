#pragma once

#include <memory>
#include <utility>

#include "drake/systems/analysis/initial_value_problem-inl.h"
#include "drake/systems/analysis/scalar_initial_value_problem.h"

namespace drake {
namespace systems {

template <typename T>
ScalarInitialValueProblem<T>::ScalarInitialValueProblem(
    const typename ScalarInitialValueProblem<T>::ScalarODEFunction&
    scalar_ode_function,
    const T& default_initial_time,
    const T& default_initial_state,
    const Parameters<T>& default_parameters) {
  // Generalizes the given scalar ODE function.
  typename InitialValueProblem<T>::ODEFunction ode_function =
      [scalar_ode_function](const T& t, const VectorBase<T>& x,
                            const Parameters<T>& k, VectorBase<T>* dx_dt) {
        dx_dt->SetAtIndex(
            0, scalar_ode_function(t, x.GetAtIndex(0), k));
      };

  // Builds a vector initial state from the given scalar one.
  const BasicVector<T> default_initial_state_as_vector(
      VectorX<T>::Constant(1, default_initial_state));

  // Instantiates the generalized initial value problem.
  generalized_ivp_ = std::make_unique<InitialValueProblem<T>>(
      ode_function, default_initial_time,
      default_initial_state_as_vector,
      default_parameters);
}

}  // namespace systems
}  // namespace drake
