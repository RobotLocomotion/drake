#include "drake/multibody/cenic/cenic_integrator.h"

#include <stdexcept>

namespace drake {
namespace multibody {

using systems::Context;
using systems::Diagram;
using systems::IntegratorBase;
using systems::System;

namespace {

template <typename T>
const MultibodyPlant<T>& GetPlantFromDiagram(const System<T>& system) {
  const auto* const diagram = dynamic_cast<const Diagram<T>*>(&system);
  if (diagram == nullptr) {
    throw std::logic_error(
        fmt::format("CenicIntegrator must be given a Diagram, not a {}",
                    NiceTypeName::Get(system)));
  }
  return diagram->template GetDowncastSubsystemByName<MultibodyPlant>("plant");
}

}  // namespace

template <typename T>
CenicIntegrator<T>::CenicIntegrator(const System<T>& system,
                                    Context<T>* context)
    : IntegratorBase<T>(system, context),
      plant_(GetPlantFromDiagram(system)),
      plant_subsystem_index_(
          static_cast<const Diagram<T>&>(system).GetSystemIndexOrAbort(
              &plant_)) {}

template <typename T>
CenicIntegrator<T>::~CenicIntegrator() = default;

template <typename T>
bool CenicIntegrator<T>::supports_error_estimation() const {
  // TODO(jwnimmer-tri) Implement me.
  return false;
}

template <typename T>
int CenicIntegrator<T>::get_error_estimate_order() const {
  // TODO(jwnimmer-tri) Implement me.
  return 0;
}

template <typename T>
void CenicIntegrator<T>::DoInitialize() {
  // TODO(jwnimmer-tri) Implement me.
  throw std::logic_error("CenicIntegrator is not implemented yet");
}

template <typename T>
bool CenicIntegrator<T>::DoStep(const T& /* h */) {
  // TODO(jwnimmer-tri) Implement me.
  throw std::logic_error("CenicIntegrator is not implemented yet");
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::CenicIntegrator);
