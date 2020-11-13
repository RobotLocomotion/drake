#include "drake/multibody/fem/dev/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
std::unique_ptr<FemState<T>> FemModel<T>::MakeFemState() const {
  std::unique_ptr<FemState<T>> state = DoMakeFemState();
  /* Set up the element cache that are compatible with the elements owned by
   this model. */
  std::vector<std::unique_ptr<ElementCacheEntry<T>>> cache;
  for (int i = 0; i < num_elements(); ++i) {
    cache.emplace_back(elements_[i]->MakeElementCacheEntry());
  }
  state->ResetElementCache(std::move(cache));
  return state;
}

/* This method assembles the element residual computed in
 FemElement::CalcResidual() into the global residual vector. */
template <typename T>
void FemModel<T>::CalcResidual(const FemState<T>& state,
                               EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual->size() == num_dofs());
  /* Verify the size of the cache in the input state is consistent with the
   number of elements in the FemModel. */
  DRAKE_DEMAND(state.element_cache_size() == num_elements());
  /* The values are accumulated in the residual, so it is important to clear
   the old data. */
  residual->setZero();
  /* The size of `element_residual` depends on the number of nodes in the
   element and may change from one element to the next. */
  VectorX<T> element_residual;
  const int solution_dim = solution_dimension();
  for (int e = 0; e < num_elements(); ++e) {
    const int element_num_nodes = elements_[e]->num_nodes();
    const int element_dofs = element_num_nodes * solution_dim;
    element_residual.resize(element_dofs);
    elements_[e]->CalcResidual(state, &element_residual);
    // TODO(xuchenhan-tri): This may become a repeating pattern. Consider
    // extracting it out into a method.
    const std::vector<NodeIndex>& element_node_indices =
        elements_[e]->node_indices();
    for (int i = 0; i < element_num_nodes; ++i) {
      for (int d = 0; d < solution_dim; ++d) {
        const int ei = element_node_indices[i];
        (*residual)[ei * solution_dim + d] +=
            element_residual[i * solution_dim + d];
      }
    }
  }
}

template <typename T>
void FemModel<T>::ThrowIfNodeIndicesAreInvalid(
    const std::set<int>& node_indices) const {
  if (node_indices.size() == 0) return;
  if (*node_indices.rbegin() != static_cast<int>(node_indices.size()) - 1 ||
      *node_indices.begin() != 0) {
    throw std::runtime_error(
        "Node indices must be consecutive and start from zero and end with the "
        "number of nodes minus one.");
  }
}

template <typename T>
void FemModel<T>::AddElement(std::unique_ptr<FemElement<T>> element) {
  DRAKE_DEMAND(element != nullptr);
  elements_.emplace_back(std::move(element));
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemModel);
