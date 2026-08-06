#include "drake/multibody/contact_solvers/icf/holonomic_constraints_data_pool.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T, int N>
HolonomicConstraintsDataPool<T, N>::~HolonomicConstraintsDataPool() = default;

template <typename T, int N>
void HolonomicConstraintsDataPool<T, N>::Resize(int num_constraints) {
  gamma_pool_.resize(num_constraints);
}

// Explicit template instantiations.
template class HolonomicConstraintsDataPool<double, 6>;
template class HolonomicConstraintsDataPool<AutoDiffXd, 6>;
template class HolonomicConstraintsDataPool<double, 3>;
template class HolonomicConstraintsDataPool<AutoDiffXd, 3>;
template class HolonomicConstraintsDataPool<double, 1>;
template class HolonomicConstraintsDataPool<AutoDiffXd, 1>;

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
