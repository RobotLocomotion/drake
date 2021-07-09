#pragma once

#include <array>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Sparse>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/fem_model_base.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {

/** %FemModel provides a fixed size implementaion of FemModelBase by
 templatizing on the type of FemElement. See FemModelBase for more information
 on the functionalities of this class. Many methods provided by FemModelBase
 (e.g. FemModelBase::CalcTangentMatrix()) involve evaluating computationally
 intensive loops over FemElement, and the overhead caused by virtual methods may
 be significant. Therefore, this class is templated on the FemElement to avoid
 the overhead of virtual methods. The type information at compile time also
 helps eliminate heap allocations.
 @tparam Element    The type of FemElement that makes up this %FemModel.
 This template parameter provides the scalar type and the compile time constants
 such as the natural, spatial and solution dimensions and the number of
 nodes/quadrature points in each element. */
template <class Element>
class FemModel : public FemModelBase<typename Element::Traits::T> {
 public:
  static_assert(
      std::is_base_of_v<FemElement<Element, typename Element::Traits>, Element>,
      "The template parameter Element should be derived from FemElement. ");
  using T = typename Element::Traits::T;
  using ElementType = Element;

  /** Creates a new FemState. The new state's number of generalized positions is
   equal to `num_dofs()`. The new state's element data is constructed using the
   FemElements owned by this %FemModel. */
  FemState<Element> MakeFemState() const {
    FemState<Element> state = DoMakeFemState();
    state.MakeElementData(elements_);
    return state;
  }

  /** The number of degrees of freedom in the model. */
  int num_dofs() const final {
    return Element::Traits::kSolutionDimension * this->num_nodes();
  }

  /** The number of FemElements owned by `this` %FemModel. */
  int num_elements() const final { return elements_.size(); }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModel);

  explicit FemModel(std::unique_ptr<internal::StateUpdater<T>> state_updater)
      : FemModelBase<T>(std::move(state_updater)) {}

  virtual ~FemModel() = default;

  /** Derived classes must implement this method to create a new FemState. */
  virtual FemState<Element> DoMakeFemState() const = 0;

  const Element& element(ElementIndex i) const {
    DRAKE_ASSERT(i.is_valid());
    DRAKE_ASSERT(i < num_elements());
    return elements_[i];
  }

  Element& mutable_element(ElementIndex i) {
    DRAKE_ASSERT(i.is_valid());
    DRAKE_ASSERT(i < num_elements());
    return elements_[i];
  }

  /** Moves the input `element` into the vector of elements held by `this`
   %FemModel. */
  void AddElement(Element&& element) {
    elements_.emplace_back(std::move(element));
  }

  /** Alternative signature for adding a new element to `this` %FemModel.
   Forwards the arguments `args` to the constructor of the Element and
   potentially creates the new element in place. */
  template <typename... Args>
  void AddElement(Args&&... args) {
    elements_.emplace_back(std::forward<Args>(args)...);
  }

  /** Adds per-vertex residuals that are explicitly specified at each vertex
   instead of accumulated from elements. The default implementation is a
   no-op. Derived classes must override this method if their residuals have
   per-vertex contribution. */
  virtual void AddExplicitResidual(EigenPtr<VectorX<T>> residual) const {
    unused(residual);
  }

 private:
  int ode_order() const final { return Element::Traits::kOdeOrder; }

  /* Implements FemModelBase::MakeFemStateBase() by simply hiding the
   result from MakeFemState() behind a unique_ptr to FemStateBase. */
  std::unique_ptr<FemStateBase<T>> DoMakeFemStateBase() const final {
    return std::make_unique<FemState<Element>>(MakeFemState());
  }

  /* Helper for DoCalcResidual(). */
  void CalcResidualForConcreteState(const FemState<Element>& state,
                                    EigenPtr<VectorX<T>> residual) const {
    DRAKE_DEMAND(residual != nullptr && residual->size() == num_dofs());
    DRAKE_DEMAND(state.element_cache_size() == num_elements());
    /* The values are accumulated in the residual, so it is important to clear
     the old data. */
    residual->setZero();
    /* Aliases to improve readability. */
    constexpr int kNumDofs = Element::Traits::kNumDofs;
    constexpr int kNumNodes = Element::Traits::kNumNodes;
    constexpr int kDim = Element::Traits::kSolutionDimension;
    /* Scratch space to store the contribution to the residual from each
     element. */
    Vector<T, kNumDofs> element_residual;
    for (ElementIndex e(0); e < num_elements(); ++e) {
      elements_[e].CalcResidual(state, &element_residual);
      const std::array<NodeIndex, kNumNodes>& element_node_indices =
          elements_[e].node_indices();
      for (int i = 0; i < kNumNodes; ++i) {
        const int ei = element_node_indices[i];
        residual->template segment<kDim>(ei * kDim) +=
            element_residual.template segment<kDim>(i * kDim);
      }
    }
    /* Add per-vertex residuals that are explicitly specified at each vertex
     instead of accumulated from elements. */
    AddExplicitResidual(residual);
  }

  /* Helper for DoCalcTangentMatrix(). */
  void CalcTangentMatrixForConcreteState(
      const FemState<Element>& state,
      Eigen::SparseMatrix<T>* tangent_matrix) const {
    DRAKE_DEMAND(tangent_matrix != nullptr &&
                 tangent_matrix->rows() == num_dofs() &&
                 tangent_matrix->cols() == num_dofs());
    DRAKE_DEMAND(state.element_cache_size() == num_elements());
    /* The values are accumulated in the tangent_matrix, so it is important to
     clear the old data. */
    using Iterator = typename Eigen::SparseMatrix<T>::InnerIterator;
    for (int k = 0; k < tangent_matrix->outerSize(); ++k) {
      for (Iterator it(*tangent_matrix, k); it; ++it) {
        it.valueRef() = 0;
      }
    }
    /* Aliases to improve readability. */
    constexpr int kNumDofs = Element::Traits::kNumDofs;
    constexpr int kNumNodes = Element::Traits::kNumNodes;
    constexpr int kDim = Element::Traits::kSolutionDimension;
    /* Scratch space to store the contribution to the tangent matrix from each
     element. */
    Eigen::Matrix<T, kNumDofs, kNumDofs> element_tangent_matrix;
    const Vector3<T>& weights = this->state_updater().weights();
    for (ElementIndex e(0); e < num_elements(); ++e) {
      element_tangent_matrix = CalcElementTangentMatrix(state, weights, e);
      const std::array<NodeIndex, kNumNodes>& element_node_indices =
          elements_[e].node_indices();
      for (int a = 0; a < kNumNodes; ++a) {
        for (int i = 0; i < kDim; ++i) {
          for (int b = 0; b < kNumNodes; ++b) {
            for (int j = 0; j < kDim; ++j) {
              // TODO(xuchenhan-tri): Compare the performance between coeffRef()
              //  and setFromTriplets() for filling out the sparse matrix.
              tangent_matrix->coeffRef(element_node_indices[a] * kDim + i,
                                       element_node_indices[b] * kDim + j) +=
                  element_tangent_matrix(a * kDim + i, b * kDim + j);
            }
          }
        }
      }
    }
  }

  /* Implements FemModelBase::SetTangentMatrixSparsityPattern(). */
  void DoSetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const final {
    DRAKE_DEMAND(tangent_matrix != nullptr);
    tangent_matrix->resize(num_dofs(), num_dofs());
    std::vector<Eigen::Triplet<T>> non_zero_entries;
    /* Alias for readability. */
    constexpr int element_num_dofs = Element::Traits::kNumDofs;
    constexpr int element_num_nodes = Element::Traits::kNumNodes;
    constexpr int kDim = Element::Traits::kSolutionDimension;
    /* Get an upper bound for the number of nonzero entries and allocate
     memories for them in the vector of triplets. */
    non_zero_entries.reserve(num_elements() * element_num_dofs *
                             element_num_dofs);
    /* Create a nonzero block for each pair of nodes that are connected by an
     edge in the mesh. */
    for (int e = 0; e < num_elements(); ++e) {
      const std::array<NodeIndex, element_num_nodes>& element_node_indices =
          elements_[e].node_indices();
      for (int a = 0; a < element_num_nodes; ++a) {
        for (int i = 0; i < kDim; ++i) {
          const int row_index = kDim * element_node_indices[a] + i;
          for (int b = 0; b < element_num_nodes; ++b) {
            for (int j = 0; j < kDim; ++j) {
              const int col_index = kDim * element_node_indices[b] + j;
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

  /* Implements FemModelBase::CalcResidual() by casting the FemStateBase
   to its concrete type. */
  void DoCalcResidual(const FemStateBase<T>& state,
                      EigenPtr<VectorX<T>> residual) const final {
    const FemState<Element>& concrete_state = cast_to_concrete_state(state);
    CalcResidualForConcreteState(concrete_state, residual);
  }

  /* Implements FemModelBase::CalcTangentMatrix() by casting the
   FemStateBase to its concrete type. */
  void DoCalcTangentMatrix(const FemStateBase<T>& state,
                           Eigen::SparseMatrix<T>* tangent_matrix) const final {
    const FemState<Element>& concrete_state = cast_to_concrete_state(state);
    CalcTangentMatrixForConcreteState(concrete_state, tangent_matrix);
  }

  /* Builds the element tangent matrix for the element with index
   `element_index` by combining the stiffness matrix, damping matrix, and the
   mass matrix according to the given `weights`.
   @param[in] state    The FemState to evaluate the residual at.
   @param[in] weights    The ordered weights to combine stiffness matrix,
   damping matrix and mass matrix into the tangent matrix.
   @param[in] element_index    Index of the element whose element tangent matrix
   is being built. */
  Eigen::Matrix<T, Element::Traits::kNumDofs, Element::Traits::kNumDofs>
  CalcElementTangentMatrix(const FemState<Element>& state,
                           const Vector3<T>& weights,
                           ElementIndex element_index) const {
    DRAKE_ASSERT(element_index.is_valid() && element_index < num_elements());
    using MatrixType =
        Eigen::Matrix<T, Element::Traits::kNumDofs, Element::Traits::kNumDofs>;
    MatrixType stiffness_matrix = MatrixType::Zero();
    elements_[element_index].CalcStiffnessMatrix(state, &stiffness_matrix);
    if constexpr (FemState<Element>::ode_order() == 0) {
      return weights[0] * stiffness_matrix;
    }
    MatrixType damping_matrix = MatrixType::Zero();
    elements_[element_index].CalcDampingMatrix(state, &damping_matrix);
    if constexpr (FemState<Element>::ode_order() == 1) {
      return weights[0] * stiffness_matrix + weights[1] * damping_matrix;
    }
    DRAKE_ASSERT(FemState<Element>::ode_order() == 2);
    MatrixType mass_matrix = MatrixType::Zero();
    elements_[element_index].CalcMassMatrix(state, &mass_matrix);
    return weights[0] * stiffness_matrix + weights[1] * damping_matrix +
           weights[2] * mass_matrix;
  }

  /* Statically cast the given FemStateBase to the FemState compatible
   with `this` FemModel.
   @pre The given `abstract_state` is compatible with the `this` FemModel. */
  const FemState<Element>& cast_to_concrete_state(
      const FemStateBase<T>& abstract_state) const {
    const auto& concrete_state =
        static_cast<const FemState<Element>&>(abstract_state);
    return concrete_state;
  }

  /* Mutable version of cast_to_concrete_state() that takes a pointer to the
   abstract state.
   @pre The given `abstract_state` is compatible with the `this` FemModel. */
  FemState<Element>& cast_to_mutable_concrete_state(
      FemStateBase<T>* abstract_state) const {
    auto* concrete_state_ptr = static_cast<FemState<Element>*>(abstract_state);
    return *concrete_state_ptr;
  }

  /* Implements FemModelBase::ThrowIfModelStateIncompatible(). */
  void ThrowIfModelStateIncompatible(
      const char* func, const FemStateBase<T>& abstract_state) const final {
    const auto* concrete_state_ptr =
        dynamic_cast<const FemState<Element>*>(&abstract_state);
    if (concrete_state_ptr == nullptr) {
      throw std::logic_error(
          std::string(func) +
          "(): The type of the FemState is incompatible with the type of the "
          "FemModel.");
    }
    if (concrete_state_ptr->num_generalized_positions() != num_dofs()) {
      throw std::logic_error(fmt::format(
          "{}(): The size of the FemState ({}) is incompatible "
          "with the size of the FemModel ({}).",
          func, concrete_state_ptr->num_generalized_positions(), num_dofs()));
    }
  }

  /* FemElements owned by this model. */
  std::vector<Element> elements_{};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
