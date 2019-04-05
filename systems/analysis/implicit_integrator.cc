#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/implicit_integrator-inl.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
template class ImplicitIntegrator<double>;
template class ImplicitIntegrator<AutoDiffXd>;
}  // namespace systems
}  // namespace drake


