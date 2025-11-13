#include "drake/multibody/contact_solvers/icf/gain_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
void GainConstraintsDataPool<T>::Resize(std::span<const int> constraint_size) {
  const int num_elements = ssize(constraint_size);
  gamma_pool_.Resize(num_elements, constraint_size, {});
  G_pool_.Resize(num_elements, constraint_size, constraint_size);

  // We will only ever update the diagonal entries in Gk, so that off-diagonal
  // entires will forever remain zero.
  const int num_constraints = constraint_size.size();
  for (int k = 0; k < num_constraints; ++k) {
    G_pool_[k].setZero();
  }
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        GainConstraintsDataPool);
