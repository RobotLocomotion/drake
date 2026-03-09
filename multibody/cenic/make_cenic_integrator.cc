#include "drake/multibody/cenic/make_cenic_integrator.h"

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/multibody/cenic/cenic_integrator.h"

namespace drake {
namespace multibody {

using systems::Context;
using systems::IntegratorBase;
using systems::System;

template <class T>
std::unique_ptr<IntegratorBase<T>> MakeCenicIntegrator(const System<T>& system,
                                                       Context<T>* context) {
  return std::make_unique<CenicIntegrator<T>>(system, context);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeCenicIntegrator<T>));

}  // namespace multibody
}  // namespace drake
