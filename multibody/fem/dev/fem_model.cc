#include "drake/multibody/fem/dev/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
std::unique_ptr<FemState<T>> FemModel<T>::MakeFemState() const {
  ThrowIfNodeIndicesAreInvalid();
  std::unique_ptr<FemState<T>> state = DoMakeFemState();
  /* Set up the element cache that are compatible with the elements owned by
   this model. */
  std::vector<std::unique_ptr<ElementCache<T>>> cache;
  for (int i = 0; i < num_elements(); ++i) {
    cache.emplace_back(elements_[i]->MakeElementCache());
  }
  state->ResetElementCache(std::move(cache));
  return state;
}

/* This method assembles the element residual computed in
 FemElement::CalcResidual() into the global residual vector. */
template <typename T>
void FemModel<T>::CalcResidual(const FemState<T>& state,
                               EigenPtr<VectorX<T>> residual) const {
  ThrowIfNodeIndicesAreInvalid();
  /* Verify the size of the cache in the input state is consistent with the
   number of elements in the FemModel. */
  DRAKE_DEMAND(state.num_element_cache() == num_elements());
  /* The values are accumulated in the residual, so it is important to clear
   the old data. */
  residual->setZero();
  VectorX<T> element_residual;
  for (int e = 0; e < num_elements(); ++e) {
    const int element_num_nodes = elements_[e]->num_nodes();
    const int element_residual_size = element_num_nodes * solution_dimension();
    if (element_residual.size() != element_residual_size) {
      element_residual.resize(element_residual_size);
    }
    elements_[e]->CalcResidual(state, &element_residual);
    // TODO(xuchenhan-tri): This may become a repeating pattern. Consider
    // extracting it out into a method.
    const std::vector<NodeIndex>& element_node_indices =
        elements_[e]->node_indices();
    for (int i = 0; i < element_num_nodes; ++i) {
      for (int d = 0; d < solution_dimension(); ++d) {
        (*residual)(element_node_indices[i] * solution_dimension() + d) +=
            element_residual(solution_dimension() * i + d);
      }
    }
  }
}

template <typename T>
void FemModel<T>::ThrowIfNodeIndicesAreInvalid() const {
  if (owned_node_indices_.size() == 0) return;
  if (*owned_node_indices_.rbegin() !=
          NodeIndex(owned_node_indices_.size() - 1) ||
      *owned_node_indices_.begin() != NodeIndex(0)) {
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

template <typename T>
void FemModel<T>::AddNodeIndex(NodeIndex i) {
  owned_node_indices_.insert(i);
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemModel);
