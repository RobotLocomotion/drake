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

/* FemModelImpl provides a fixed size implementaion of FemModel by
 templatizing on the type of FemElement. See FemModel for more information
 on the user-facing APIs of this class. Many methods provided by FemModel
 (e.g. FemModel::CalcTangentMatrix) involve evaluating computationally
 intensive loops over FemElement, and the overhead caused by virtual methods may
 be significant. Therefore, this class is templated on the FemElement to avoid
 the overhead of virtual methods. The type information at compile time also
 helps eliminate heap allocations.
 @tparam Element  The type of FEM elements that makes up this FemModelImpl.
 This template parameter must be an instantiation of FemElement, which provides
 the scalar type and the compile time constants such as the natural dimension
 and the number of nodes/quadrature points in each element. See FemElements for
 more details. */
template <class Element>
class FemModelImpl : public FemModel<typename Element::T> {
 public:
  static_assert(
      std::is_base_of_v<FemElement<Element>, Element>,
      "The template parameter Element should be derived from FemElement. ");
  using T = typename Element::T;
  using Data = typename Element::Data;

  /* Returns the number of FEM elements owned by this FEM model. */
  int num_elements() const final { return elements_.size(); }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModelImpl);

  /* Creates an empty FemModelImpl with no elements. */
  FemModelImpl() = default;

  ~FemModelImpl() = default;

  /* Moves the input `element` into the vector of elements held by this
   FemModelImpl. */
  void AddElement(Element&& element) {
    elements_.emplace_back(std::move(element));
  }

  /* Alternative signature for adding a new element to this FemModelImpl.
   Forwards the arguments `args` to the constructor of the Element and
   potentially creates the new element in place. */
  template <typename... Args>
  void AddElement(Args&&... args) {
    elements_.emplace_back(std::forward<Args>(args)...);
  }

 private:
  void DoCalcResidual(const FemState<T>& fem_state,
                      EigenPtr<VectorX<T>> residual) const final {
    /* The values are accumulated in the residual, so it is important to clear
     the old data. */
    residual->setZero();
    /* Aliases to improve readability. */
    constexpr int num_dofs = Element::num_dofs;
    constexpr int num_nodes = Element::num_nodes;
    constexpr int kDim = 3;
    /* Scratch space to store the contribution to the residual from each
     element. */
    Vector<T, num_dofs> element_residual;
    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(element_data_index_);
    for (int e = 0; e < num_elements(); ++e) {
      elements_[e].CalcResidual(element_data[e], &element_residual);
      const std::array<FemNodeIndex, num_nodes>& element_node_indices =
          elements_[e].node_indices();
      for (int i = 0; i < num_nodes; ++i) {
        const int ei = element_node_indices[i];
        residual->template segment<kDim>(ei * kDim) +=
            element_residual.template segment<kDim>(i * kDim);
      }
    }
  }

  void DoCalcTangentMatrix(
      const FemState<T>& fem_state, const Vector3<T>& weights,
      PetscSymmetricBlockSparseMatrix* tangent_matrix) const {
    /* Clears the old data. */
    tangent_matrix->SetZero();

    /* Aliases to improve readability. */
    constexpr int num_dofs = Element::num_dofs;
    constexpr int num_nodes = Element::num_nodes;

    /* Scratch space to store the contribution to the tangent matrix from each
     element. */
    Vector<int, num_nodes> block_indices;
    const std::vector<Data>& element_data =
        fem_state.template EvalElementData<Data>(element_data_index_);
    Eigen::Matrix<T, num_dofs, num_dofs> element_tangent_matrix;
    for (int e = 0; e < num_elements(); ++e) {
      elements_[e].CalcTangentMatrix(element_data[e], weights,
                                     &element_tangent_matrix);
      const std::array<FemNodeIndex, num_nodes>& element_node_indices =
          elements_[e].node_indices();
      // TODO(xuchenhan-tri): Avoid this index copy.
      for (int a = 0; a < num_nodes; ++a) {
        block_indices(a) = element_node_indices[a];
      }
      tangent_matrix->AddToBlock(block_indices, element_tangent_matrix);
    }
  }

  std::unique_ptr<PetscSymmetricBlockSparseMatrix>
  DoMakePetscSymmetricBlockSparseTangentMatrix() const final {
    std::vector<std::unordered_set<int>> neighbor_nodes(this->num_nodes());
    /* Alias for readability. */
    constexpr int element_num_nodes = Element::num_nodes;
    constexpr int element_num_dofs = Element::num_dofs;
    constexpr int kDim = 3;
    /* Create a nonzero block for each pair of nodes that are connected by an
     edge in the mesh. */
    for (int e = 0; e < num_elements(); ++e) {
      const std::array<FemNodeIndex, element_num_nodes>& element_node_indices =
          elements_[e].node_indices();
      for (int a = 0; a < element_num_nodes; ++a) {
        for (int b = a; b < element_num_nodes; ++b) {
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

    /* Populate the tangent matrix with zeros at appropriate places to allocate
     memory. */
    Vector<int, element_num_nodes> block_indices;
    const Eigen::Matrix<double, element_num_dofs, element_num_dofs>
        zero_matrix =
            Eigen::Matrix<double, element_num_dofs, element_num_dofs>::Zero();
    for (int e = 0; e < num_elements(); ++e) {
      const std::array<FemNodeIndex, element_num_nodes>& element_node_indices =
          elements_[e].node_indices();
      // TODO(xuchenhan-tri): Here we are relying on the implicit assumption
      //  that all node indices are contiguous from 0 to the number of nodes. We
      //  should at least verify this assumption and abort if this assumption is
      //  violated.
      // TODO(xuchenhan-tri): Avoid this index copy.
      for (int a = 0; a < element_num_nodes; ++a) {
        block_indices(a) = element_node_indices[a];
      }
      tangent_matrix->AddToBlock(block_indices, zero_matrix);
    }
    return tangent_matrix;
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
    for (FemElementIndex i(0); i < num_elements(); ++i) {
      (*data)[i] = elements_[i].ComputeData(fem_state);
    }
  }

  /* FemElements owned by this model. */
  std::vector<Element> elements_{};
  systems::CacheIndex element_data_index_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
