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

/* This method assembles the element tangent matrix computed in
 FemElement::CalcTangentMatrix() into the global tangent matrix. */
template <typename T>
void FemModel<T>::CalcTangentMatrix(
    const FemState<T>& state, Eigen::SparseMatrix<T>* tangent_matrix) const {
  /* Verify the size of the cache in the input state is consistent with the
   number of elements in the FemModel. */
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(state.element_cache_size() == num_elements());
  /* The values are accumulated in the tangent_matrix, so it is important to
   clear the old data. */
  for (int k = 0; k < tangent_matrix->outerSize(); ++k)
    for (typename Eigen::SparseMatrix<T>::InnerIterator it(*tangent_matrix, k);
         it; ++it) {
      it.valueRef() = 0.0;
    }
  MatrixX<T> element_tangent_matrix;
  const int solution_dim = solution_dimension();
  for (int e = 0; e < num_elements(); ++e) {
    const int element_num_nodes = elements_[e]->num_nodes();
    const int element_dof = elements_[e]->num_dofs();
    element_tangent_matrix.resize(element_dof, element_dof);
    elements_[e]->CalcTangentMatrix(state, &element_tangent_matrix);
    // TODO(xuchenhan-tri): This may become a repeating pattern. Consider
    // extracting it out into a method.
    const std::vector<NodeIndex>& element_node_indices =
        elements_[e]->node_indices();
    for (int a = 0; a < element_num_nodes; ++a) {
      for (int i = 0; i < solution_dim; ++i) {
        for (int b = 0; b < element_num_nodes; ++b) {
          for (int j = 0; j < solution_dim; ++j) {
            // TODO(xuchenhan-tri): Compare the performance between coeffRef()
            // and setFromTriplets() for filling out the sparse matrix.
            tangent_matrix->coeffRef(
                element_node_indices[a] * solution_dim + i,
                element_node_indices[b] * solution_dim + j) +=
                element_tangent_matrix(a * solution_dim + i,
                                       b * solution_dim + j);
          }
        }
      }
    }
  }
}

template <typename T>
void FemModel<T>::SetTangentMatrixSparsityPattern(
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  /* Get an upper bound for the number of nonzero entries. */
  tangent_matrix->resize(num_dofs(), num_dofs());
  std::vector<Eigen::Triplet<T>> non_zero_entries;
  int nnz = 0;
  const int solution_dim = solution_dimension();
  for (int e = 0; e < num_elements(); ++e) {
    const int element_num_nodes = elements_[e]->num_nodes();
    const int element_dof = element_num_nodes * solution_dim;
    /* Add in the maximum number of nonzero entries that can be contributed by
     this element. There are likely node-pairs that appear in more than one
     element that get counted multiple times, but that ok because duplicate
     triplets will only be allocated once by Eigen::SparseMatrix. */
    nnz += element_dof * element_dof;
  }
  non_zero_entries.reserve(nnz);
  for (int e = 0; e < num_elements(); ++e) {
    const std::vector<NodeIndex>& element_node_indices =
        elements_[e]->node_indices();
    const int element_num_nodes = elements_[e]->num_nodes();
    for (int a = 0; a < element_num_nodes; ++a) {
      for (int i = 0; i < 3; ++i) {
        int row_index = solution_dim * element_node_indices[a] + i;
        for (int b = 0; b < element_num_nodes; ++b) {
          for (int j = 0; j < 3; ++j) {
            int col_index = solution_dim * element_node_indices[b] + j;
            non_zero_entries.emplace_back(row_index, col_index, 0);
          }
        }
      }
    }
  }
  tangent_matrix->setFromTriplets(non_zero_entries.begin(),
                                  non_zero_entries.end());
  tangent_matrix->makeCompressed();
}

template <typename T>
void FemModel<T>::ThrowIfNodeIndicesAreInvalid(
    const std::set<int>& node_indices) const {
  if (node_indices.size() == 0) return;
  if (*node_indices.rbegin() != static_cast<int>(node_indices.size()) - 1 ||
      *node_indices.begin() != 0) {
    throw std::runtime_error(
        "Node indices must be consecutive and start from zero and end with "
        "the number of nodes minus one.");
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
