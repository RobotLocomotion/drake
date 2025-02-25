#include "drake/systems/estimators/gaussian_state_observer.h"

namespace drake {
namespace systems {
namespace estimators {

template <typename T>
GaussianStateObserver<T>::GaussianStateObserver() {}

template <typename T>
GaussianStateObserver<T>::GaussianStateObserver(SystemScalarConverter converter)
    : LeafSystem<T>(converter) {}

template <typename T>
GaussianStateObserver<T>::~GaussianStateObserver() = default;

}  // namespace estimators
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::estimators::GaussianStateObserver);
