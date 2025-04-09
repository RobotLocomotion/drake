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
void Sdirk2Integrator<T>::DoInitialize() {
  using std::isnan;

  // Set default accuracy
  const double kDefaultAccuracy = 1e-1;

  // Set an artificial step size target, if not set already.
  if (isnan(this->get_initial_step_size_target())) {
    // Verify that maximum step size has been set.
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");

    this->request_initial_step_size_target(this->get_maximum_step_size());
  }

  // Allocate intermediate variables
  const int nx = this->get_system().num_continuous_states();
  x_.resize(nx);
  k1_.resize(nx);
  k2_.resize(nx);

  // Set the working accuracy to a reasonable default
  double working_accuracy = this->get_target_accuracy();
  if (isnan(working_accuracy)) working_accuracy = kDefaultAccuracy;
  this->set_accuracy_in_use(working_accuracy);
}

template <class T>
bool Sdirk2Integrator<T>::DoImplicitIntegratorStep(const T& h) {
  (void)h;
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::Sdirk2Integrator);