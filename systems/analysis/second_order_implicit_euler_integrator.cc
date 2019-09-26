#include "drake/systems/analysis/second_order_implicit_euler_integrator.h"
#include "drake/systems/analysis/second_order_implicit_euler_integrator-inl.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
template class SecondOrderImplicitEulerIntegrator<double>;
template class SecondOrderImplicitEulerIntegrator<AutoDiffXd>;
}  // namespace systems
}  // namespace drake


