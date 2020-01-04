#include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
template class VelocityImplicitEulerIntegrator<double>;
template class VelocityImplicitEulerIntegrator<AutoDiffXd>;
}  // namespace systems
}  // namespace drake


