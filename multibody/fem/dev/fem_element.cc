#include "drake/multibody/fem/dev/fem_element.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
void FemElement<T>::CalcResidual(const FemState<T>& state,
                                 EigenPtr<VectorX<T>> residual) const {
  DRAKE_ASSERT(residual != nullptr);
  DRAKE_ASSERT(residual->size() == solution_dimension() * num_nodes());
  DoCalcResidual(state, residual);
}

template <typename T>
FemElement<T>::FemElement(ElementIndex element_index,
                          const std::vector<NodeIndex>& node_indices)
    : element_index_(element_index), node_indices_(node_indices) {
  DRAKE_ASSERT(element_index_.is_valid());
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemElement);
