#include "drake/multibody/contact_solvers/icf/gain_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
GainConstraintsDataPool<T>::~GainConstraintsDataPool() = default;

template <typename T>
void GainConstraintsDataPool<T>::Resize(std::span<const int> constraint_size) {
  const int num_elements = ssize(constraint_size);
  gamma_pool_.Resize(num_elements, constraint_size);
  G_pool_.Resize(num_elements, constraint_size);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        GainConstraintsDataPool);
