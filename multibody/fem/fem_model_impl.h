#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/fem_element.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* See FemModel for documentation of this class.
 @tparam Element  The type of FEM elements that makes up this FemModelImpl.
 This template parameter must be an instantiation of FemElement, which provides
 the scalar type and the compile time constants such as the natural dimension
 and the number of nodes/quadrature points in each element. See FemElements for
 more details. */
template <class Element>
class FemModelImpl : public FemModel<typename Element::T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModelImpl);

  static_assert(
      std::is_base_of_v<FemElement<Element>, Element>,
      "The template parameter Element should be derived from FemElement. ");
  using T = typename Element::T;
  using Data = typename Element::Data;

  /* Returns the number of FEM elements owned by this FEM model. */
  int num_elements() const final { return elements_.size(); }

 protected:
  /* Creates an empty FemModelImpl with no elements. */
  FemModelImpl() = default;

  ~FemModelImpl() = default;

  /* Moves the input `element` into the vector of elements held by this
   FemModelImpl. */
  void AddElement(Element&& element) {
    elements_.emplace_back(std::move(element));
  }

  /* Moves the input `elements`' entries into the vector of elements owned by
   this FemModelImpl. The entries in `elements` will be left in the moved-from
   state, but the size of `elements` does not change.
   @pre elements != nullptr */
  void AddElements(std::vector<Element>* elements) {
    DRAKE_DEMAND(elements != nullptr);
    elements_.insert(elements_.end(),
                     std::make_move_iterator(elements->begin()),
                     std::make_move_iterator(elements->end()));
  }

  /* Returns all elements stored in this model. */
  const std::vector<Element>& elements() const { return elements_; }

 protected:
  /** Returns the cache index for the per-element data in this model. */
  systems::CacheIndex element_data_index() const { return element_data_index_; }

 private:
  void DoCalcResidual(const FemState<T>& fem_state,
                      EigenPtr<VectorX<T>> residual) const final {
    /* The values are accumulated in the residual, so it is important to clear
     the old data. */
    residual->setZero();
    constexpr int kDim = 3;
    /* Scratch space to store the contribution to the residual from each
     element. */
    Vector<T, Element::num_dofs> element_residual;
    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(element_data_index_);
    for (int e = 0; e < num_elements(); ++e) {
      /* residual = Ma-fₑ(x)-fᵥ(x, v)-fₑₓₜ. */
      /* The Ma-fₑ(x)-fᵥ(x, v) term. */
      elements_[e].CalcInverseDynamics(element_data[e], &element_residual);
      /* The -fₑₓₜ term. Currently the only type of external force is gravity.
       */
      elements_[e].AddScaledGravityForce(
          element_data[e], -1.0, this->gravity_vector(), &element_residual);
      const std::array<FemNodeIndex, Element::num_nodes>& element_node_indices =
          elements_[e].node_indices();
      for (int a = 0; a < Element::num_nodes; ++a) {
        const int global_node = element_node_indices[a];
        residual->template segment<kDim>(global_node * kDim) +=
            element_residual.template segment<kDim>(a * kDim);
      }
    }
  }

  void DoCalcTangentMatrix(
      const FemState<T>& fem_state, const Vector3<T>& weights,
      PetscSymmetricBlockSparseMatrix* tangent_matrix) const final {
    /* We already check for the scalar type in `CalcTangentMatrix()` but the `if
     constexpr` here is still needed to make the compiler happy. */
    if constexpr (std::is_same_v<T, double>) {
      /* Clears the old data. */
      tangent_matrix->SetZero();

      /* Scratch space to store the contribution to the tangent matrix from each
       element. */
      Vector<int, Element::num_nodes> block_indices;
      const std::vector<Data>& element_data =
          fem_state.template EvalElementData<Data>(element_data_index_);
      Eigen::Matrix<T, Element::num_dofs, Element::num_dofs>
          element_tangent_matrix;
      for (int e = 0; e < num_elements(); ++e) {
        elements_[e].CalcTangentMatrix(element_data[e], weights,
                                       &element_tangent_matrix);
        const std::array<FemNodeIndex, Element::num_nodes>&
            element_node_indices = elements_[e].node_indices();
        for (int a = 0; a < Element::num_nodes; ++a) {
          block_indices(a) = element_node_indices[a];
        }
        tangent_matrix->AddToBlock(block_indices, element_tangent_matrix);
      }
    } else {
      DRAKE_UNREACHABLE();
    }
  }

  std::unique_ptr<PetscSymmetricBlockSparseMatrix>
  DoMakePetscSymmetricBlockSparseTangentMatrix() const final {
    /* We already check for the scalar type in `CalcTangentMatrix()` but the `if
     constexpr` here is still needed to make the compiler happy. */
    if constexpr (std::is_same_v<T, double>) {
      std::vector<std::unordered_set<int>> neighbor_nodes(this->num_nodes());
      constexpr int kDim = 3;
      /* Create a nonzero block for each pair of nodes that are connected by an
       edge in the mesh. */
      for (int e = 0; e < num_elements(); ++e) {
        const std::array<FemNodeIndex, Element::num_nodes>&
            element_node_indices = elements_[e].node_indices();
        for (int a = 0; a < Element::num_nodes; ++a) {
          for (int b = a; b < Element::num_nodes; ++b) {
            /* PetscSymmetricBlockSparseMatrix only needs to allocate for the
             upper triangular part of the matrix. So instead of allocating for
             both (element_node_indices[a], element_node_indices[b]) and
             (element_node_indices[b], element_node_indices[a]) blocks, we only
             allocate for one of them. See PetscSymmetricBlockSparseMatrix. */
            const int block_row =
                std::min(element_node_indices[a], element_node_indices[b]);
            const int block_col =
                std::max(element_node_indices[a], element_node_indices[b]);
            neighbor_nodes[block_row].insert(block_col);
          }
        }
      }
      std::vector<int> nonzero_blocks(this->num_nodes());
      for (int i = 0; i < this->num_nodes(); ++i) {
        nonzero_blocks[i] = neighbor_nodes[i].size();
      }
      auto tangent_matrix = std::make_unique<PetscSymmetricBlockSparseMatrix>(
          this->num_dofs(), kDim, nonzero_blocks);

      /* Populate the tangent matrix with zeros at appropriate places to
       allocate memory. */
      Vector<int, Element::num_nodes> block_indices;
      const Eigen::Matrix<double, Element::num_dofs, Element::num_dofs>
          zero_matrix = Eigen::Matrix<double, Element::num_dofs,
                                      Element::num_dofs>::Zero();
      for (int e = 0; e < num_elements(); ++e) {
        const std::array<FemNodeIndex, Element::num_nodes>&
            element_node_indices = elements_[e].node_indices();
        // TODO(xuchenhan-tri): Here we are relying on the implicit assumption
        //  that all node indices are contiguous from 0 to the number of nodes.
        //  We should at least verify this assumption and abort if this
        //  assumption is violated.
        for (int a = 0; a < Element::num_nodes; ++a) {
          block_indices(a) = element_node_indices[a];
        }
        tangent_matrix->AddToBlock(block_indices, zero_matrix);
      }
      return tangent_matrix;
    } else {
      DRAKE_UNREACHABLE();
    }
  }

  void DeclareCacheEntries(
      internal::FemStateSystem<T>* fem_state_system) final {
    element_data_index_ =
        fem_state_system
            ->DeclareCacheEntry(
                "element data",
                systems::ValueProducer(this,
                                       &FemModelImpl<Element>::CalcElementData))
            .cache_index();
  }

  /* Computes the element data for each element in this FEM model. */
  void CalcElementData(const systems::Context<T>& context,
                       std::vector<Data>* data) const {
    DRAKE_DEMAND(data != nullptr);
    data->resize(num_elements());
    const FemState<T> fem_state(&(this->fem_state_system()), &context);
    for (int i = 0; i < num_elements(); ++i) {
      (*data)[i] = elements_[i].ComputeData(fem_state);
    }
  }

  /* FemElements owned by this model. */
  std::vector<Element> elements_;
  systems::CacheIndex element_data_index_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
