#include "drake/systems/analysis/scalar_initial_value_problem.h"

namespace drake {
namespace systems {

template <typename T>
ScalarInitialValueProblem<T>::ScalarInitialValueProblem(
    const ScalarOdeFunction& scalar_ode_function, const T& x0,
    const Eigen::Ref<const VectorX<T>>& k) {
  // Wraps the given scalar ODE function as a vector ODE function.
  typename InitialValueProblem<T>::OdeFunction ode_function =
      [scalar_ode_function](const T& t, const VectorX<T>& x,
                            const VectorX<T>& params) -> VectorX<T> {
    return VectorX<T>::Constant(1, scalar_ode_function(t, x[0], params));
  };
  // Instantiates the vector initial value problem.
  vector_ivp_ =
      std::make_unique<InitialValueProblem<T>>(ode_function, Vector1<T>{x0}, k);
}

template <typename T>
T ScalarInitialValueProblem<T>::Solve(const T& t0, const T& tf) const {
  return this->vector_ivp_->Solve(t0, tf)[0];
}

template <typename T>
std::unique_ptr<ScalarDenseOutput<T>> ScalarInitialValueProblem<T>::DenseSolve(
    const T& t0, const T& tf) const {
  // Delegates request to the vector form of this IVP by putting
  // specified values in vector form and the resulting dense output
  // back into scalar form.
  const int kDimension = 0;
  std::unique_ptr<DenseOutput<T>> vector_dense_output =
      this->vector_ivp_->DenseSolve(t0, tf);
  return std::make_unique<ScalarViewDenseOutput<T>>(
      std::move(vector_dense_output), kDimension);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ScalarInitialValueProblem);
