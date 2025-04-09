#include "drake/systems/analysis/sdirk2_integrator.h"

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
Sdirk2Integrator<T>::~Sdirk2Integrator() = default;

template <class T>
std::unique_ptr<ImplicitIntegrator<T>>
Sdirk2Integrator<T>::DoImplicitIntegratorClone() const {
  return std::make_unique<Sdirk2Integrator>(this->get_system());
}

template <class T>
void Sdirk2Integrator<T>::DoResetImplicitIntegratorStatistics() {
  num_nr_iterations_ = 0;
}

template <class T>
void Sdirk2Integrator<T>::DoResetCachedJacobianRelatedMatrices() {
 iteration_matrix_ = {};
}

template <class T>
void Sdirk2Integrator<T>::DoInitialize() {}

template <class T>
bool Sdirk2Integrator<T>::DoImplicitIntegratorStep(const T& h) {
  (void)h;
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::Sdirk2Integrator);