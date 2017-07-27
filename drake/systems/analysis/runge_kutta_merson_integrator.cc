#include "drake/systems/analysis/runge_kutta_merson_integrator.h"
#include "drake/systems/analysis/runge_kutta_merson_integrator-inl.h"

#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace systems {
template class RungeKuttaMersonIntegrator<double>;
template class RungeKuttaMersonIntegrator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake


