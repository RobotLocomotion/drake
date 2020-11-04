#include "drake/multibody/fem/dev/fem_element.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
void FemElement<T>::CalcResidual(const FemState<T>& s,
                                 EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  DRAKE_DEMAND(residual->size() == num_problem_dim() * num_nodes());
  DoCalcResidual(s, residual);
}

template <typename T>
FemElement<T>::FemElement(ElementIndex element_index,
                          const std::vector<NodeIndex>& node_indices)
    : element_index_(element_index), node_indices_(node_indices) {
  DRAKE_DEMAND(element_index_.is_valid());
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemElement);
