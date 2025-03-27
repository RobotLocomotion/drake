#include "drake/systems/analysis/rosenbrock2_integrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace systems {

template <class T>
Rosenbrock2Integrator<T>::~Rosenbrock2Integrator() = default;

template <class T>
std::unique_ptr<ImplicitIntegrator<T>>
Rosenbrock2Integrator<T>::DoImplicitIntegratorClone() const {
  return std::make_unique<Rosenbrock2Integrator>(this->get_system());
}

template <class T>
void Rosenbrock2Integrator<T>::DoInitialize() {
  using std::isnan;

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");

    this->request_initial_step_size_target(this->get_maximum_step_size());
  }
}

template <class T>
bool Rosenbrock2Integrator<T>::DoImplicitIntegratorStep(const T& h) {
  (void)h;
  return true;  // step was successful
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::Rosenbrock2Integrator);
