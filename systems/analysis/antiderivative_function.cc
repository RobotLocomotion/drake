#include "drake/systems/analysis/antiderivative_function.h"

namespace drake {
namespace systems {

template <typename T>
AntiderivativeFunction<T>::AntiderivativeFunction(
    const IntegrableFunction& integrable_function,
    const Eigen::Ref<const VectorX<T>>& k) {
  // Expresses the scalar integral to be solved as an ODE.
  typename ScalarInitialValueProblem<T>::ScalarOdeFunction scalar_ode_function =
      [integrable_function](const T& t, const T& x,
                            const VectorX<T>& params) -> T {
    unused(x);
    return integrable_function(t, params);
  };

  // Instantiates the scalar initial value problem.
  scalar_ivp_ = std::make_unique<ScalarInitialValueProblem<T>>(
      scalar_ode_function, 0.0, k);
}

template <typename T>
T AntiderivativeFunction<T>::Evaluate(const T& v, const T& u) const {
  return this->scalar_ivp_->Solve(v, u);
}

template <typename T>
std::unique_ptr<ScalarDenseOutput<T>>
AntiderivativeFunction<T>::MakeDenseEvalFunction(const T& v, const T& w) const {
  return this->scalar_ivp_->DenseSolve(v, w);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::AntiderivativeFunction);
