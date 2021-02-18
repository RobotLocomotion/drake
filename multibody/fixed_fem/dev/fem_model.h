#pragma once

#include <array>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Sparse>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/dirichlet_boundary_condition.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/state_updater.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

template <typename T>
class AbstractFemModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractFemModel);
  AbstractFemModel() = default;
  virtual ~AbstractFemModel() = default;

  /** The number of nodes that are associated with `this` %FemModel. */
  int num_nodes() const { return num_nodes_; }

  /** The number of degrees of freedom in the model. */
  virtual int num_dofs() const = 0;

  /** Calculates the residual at the given AbstractFemState by assembling the
  residual from each element. Suppose the linear or nonlinear system generated
  from the FEM discretization is G(z) = 0, then the output `residual` is equal
  to the function G evaluated at the input `state`. */
  virtual void CalcResidual(const AbstractFemState<T>& state,
                            EigenPtr<VectorX<T>> residual) const = 0;

  /** Calculates the tangent matrix at the given AbstractFemState. The ij-th
   entry of the tangent matrix is the derivative of the i-th entry of the
   residual (calculated by CalcResidual()) with respect to the j-th generalized
   unknown zⱼ.
   @param[in] state    The AbstractFemState at which the residual is evaluated.
   @param[out] tangent_matrix    The output tangent_matrix. Suppose the linear
   or nonlinear system generated from the FEM discretization is G(z) = 0, then
   `tangent_matrix` is equal to ∇G evaluated at the input `state`.
   @pre tangent_matrix != nullptr.
   @pre The size of the matrix behind `tangent_matrix` is `num_dofs()` *
   `num_dofs()`. */
  virtual void CalcTangentMatrix(
      const AbstractFemState<T>& state,
      Eigen::SparseMatrix<T>* tangent_matrix) const = 0;

  /** Sets the sparsity pattern for the tangent matrix of this
   %AbstractFemModel.
   @param[out] tangent_matrix    The tangent matrix of this %AbstractFemModel.
   Its size and sparsity pattern will be set and it will be ready to be passed
   into CalcTangentMatrix().
   @pre `tangent_matrix` must not be the null pointer. */
  virtual void SetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const = 0;

  /** Updates the AbstractFemState `state` given the change in the value of the
   highest order state `z`.
   @pre state != nullptr.
   @pre dz.size() == state.num_generalized_positions(). */
  void UpdateState(const VectorX<T>& dz, AbstractFemState<T>* state) const {
    DRAKE_DEMAND(state != nullptr);
    DRAKE_DEMAND(dz.size() == state->num_generalized_positions());
    DoUpdateState(dz, state);
  }

  /** Advance the given `state` by one time step.
   If `this` FemModel is zeroth_order (see FemState::ode_order()), throw an
   exception.
   @param[in] prev_state The state at the previous time step.
   @param[in, out] next_state The state at the new time step.
   @pre next_state != nullptr.
   @pre next_state->num_generalized_positions() ==
   prev_state.num_generalized_positions(). */
  void AdvanceOneTimeStep(const AbstractFemState<T>& prev_state,
                          AbstractFemState<T>* next_state) const {
    DRAKE_DEMAND(next_state != nullptr);
    DRAKE_DEMAND(prev_state.num_generalized_positions() ==
                 next_state->num_generalized_positions());
    DoAdvanceOneTimeStep(prev_state, next_state);
  }

  virtual void ApplyBoundaryConditions(AbstractFemState<T>* state) const = 0;

 protected:
  /* Derived classes must override this method to provide an implementation for
   the NVI UpdateState(). */
  virtual void DoUpdateState(const VectorX<T>& dz,
                             AbstractFemState<T>* state) const = 0;

  /* Derived classes must override this method to provide an implementation for
   the NVI AdvanceOneTimeStep(). */
  virtual void DoAdvanceOneTimeStep(const AbstractFemState<T>& prev_state,
                                    AbstractFemState<T>* next_state) const = 0;

 protected:
  void increment_num_nodes(int num_new_nodes) { num_nodes_ += num_new_nodes; }

 private:
  /* The total number of nodes in the system. */
  int num_nodes_{0};
};

/** %FemModel calculates the components of the discretized FEM equations.
 Suppose the PDE at hand is of the form

     F(z, ∇z, ...) = 0.

 where ... indicates possible higher derivatives that we aren't concerned with
 here. In this PDE, z: Ω ⊂ Rᴰ → Rᵈ, is the unknown function being solved for.
 Here, Ω is the domain, D is the dimension of the domain, and d is the
 solution dimension. For instance, if you are solving for the temperature of a
 3D object, then the domain is three-dimensional (D = 3), while the solution,
 which is the temperature at a point within the object, is one-dimensional (d
 = 1). After spatial and time discretization, the PDE is reduced to a system
 of linear or nonlinear equations of the form:

     G(z₁, z₂, ..., zₙ) = 0,

 where n is the number of nodes in the discretization and G is a function from
 Rⁿᵈ to Rⁿᵈ. The linear or nonlinear equation in the system associated with
 the node `a` has the form

     Gₐ(z₁, z₂, ..., zₙ) = 0,

 where Gₐ is a function from Rⁿᵈ → Rᵈ and a = 1, ..., n. The nodal values z₁,
 z₂, ..., zₙ are solved for with a linear or nonlinear solver, and the
 solution z is interpolated from these nodal values.

 %FemModel calculates various components of the system of linear or nonlinear
 equations that supports solving the system. For example, CalcResidual()
 calculates the value of G evaluated at the current state and
 CalcTangentMatrix() calculates ∇G at the current state.
 @tparam Element    The type of FemElement that makes up this %FemModel.
 This template parameter provides the scalar type and the compile time
 constants such as the natural, spatial and solution dimensions and the number
 of nodes/quadrature points in each element. */
template <class Element>
class FemModel : public AbstractFemModel<typename Element::Traits::T> {
 public:
  static_assert(
      std::is_base_of_v<FemElement<Element, typename Element::Traits>, Element>,
      "The template parameter Element should be derived from FemElement. ");
  using T = typename Element::Traits::T;
  using ElementType = Element;

  /** Creates a new FemState. The new state's number of generalized positions
   is equal to `num_dofs()`. The new state's element data is constructed using
   the FemElements owned by this %FemModel. */
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
  int num_elements() const { return elements_.size(); }

  /** Calculates the residual at the given FemState by assembling the residual
   from each element. Suppose the linear or nonlinear system generated from
   the FEM discretization is G(z) = 0, then the output `residual` is equal to
   the function G evaluated at the input `state`.
   @param[in] state The FemState at which to evaluate the residual.
   @param[out] residual The output residual evaluated at `state`.
   @pre residual != nullptr.
   @pre The size of the vector behind `residual` must be `num_dofs()`. */
  void CalcResidual(const FemState<Element>& state,
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

    if (dirichlet_bc_ != nullptr) {
      dirichlet_bc_->ApplyBcToResidual(residual);
    }
  }

  /** Calculates the tangent matrix at the given FemState. The ij-th entry of
   the tangent matrix is the derivative of the i-th entry of the residual
   (calculated by CalcResidual()) with respect to the j-th generalized unknown
   zⱼ.
   @param[in] state    The FemState at which the residual is evaluated.
   @param[out] tangent_matrix    The output tangent_matrix. Suppose the linear
   or nonlinear system generated from the FEM discretization is G(z) = 0, then
   `tangent_matrix` is equal to ∇G evaluated at the input `state`.
   @pre tangent_matrix != nullptr.
   @pre The size of the matrix behind `tangent_matrix` is `num_dofs()` *
   `num_dofs()`. */
  void CalcTangentMatrix(const FemState<Element>& state,
                         Eigen::SparseMatrix<T>* tangent_matrix) const {
    DRAKE_DEMAND(tangent_matrix != nullptr &&
                 tangent_matrix->rows() == num_dofs() &&
                 tangent_matrix->cols() == num_dofs());
    DRAKE_DEMAND(state.element_cache_size() == num_elements());
    /* The values are accumulated in the tangent_matrix, so it is important to
     clear the old data. */
    tangent_matrix->setZero();
    /* Aliases to improve readability. */
    constexpr int kNumDofs = Element::Traits::kNumDofs;
    constexpr int kNumNodes = Element::Traits::kNumNodes;
    constexpr int kDim = Element::Traits::kSolutionDimension;
    /* Scratch space to store the contribution to the tangent matrix from each
     element. */
    Eigen::Matrix<T, kNumDofs, kNumDofs> element_tangent_matrix;
    const Vector3<T>& weights = state_updater_->weights();
    for (ElementIndex e(0); e < num_elements(); ++e) {
      element_tangent_matrix = CalcElementTangentMatrix(state, weights, e);
      const std::array<NodeIndex, kNumNodes>& element_node_indices =
          elements_[e].node_indices();
      for (int a = 0; a < kNumNodes; ++a) {
        for (int i = 0; i < kDim; ++i) {
          for (int b = 0; b < kNumNodes; ++b) {
            for (int j = 0; j < kDim; ++j) {
              // TODO(xuchenhan-tri): Compare the performance between
              // coeffRef()
              //  and setFromTriplets() for filling out the sparse matrix.
              tangent_matrix->coeffRef(element_node_indices[a] * kDim + i,
                                       element_node_indices[b] * kDim + j) +=
                  element_tangent_matrix(a * kDim + i, b * kDim + j);
            }
          }
        }
      }
    }
    if (dirichlet_bc_ != nullptr) {
      dirichlet_bc_->ApplyBcToTangentMatrix(tangent_matrix);
    }
  }

  /** Implemenets AbstractFemModel::SetTangentMatrixSparsityPattern(). */
  void SetTangentMatrixSparsityPattern(
      Eigen::SparseMatrix<T>* tangent_matrix) const override {
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

  /** Takes ownership of the given Dirichlet boundary condition and apply it
   when the model is evaluated. */
  void SetDirichletBoundaryCondition(
      std::unique_ptr<DirichletBoundaryCondition<FemState<Element>>>
          dirichlet_bc) {
    dirichlet_bc_ = std::move(dirichlet_bc);
  }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModel);

  explicit FemModel(
      std::unique_ptr<StateUpdater<FemState<Element>>> state_updater)
      : state_updater_(std::move(state_updater)) {}

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
  virtual void AddExplicitResidual(EigenPtr<VectorX<T>> residual) const {}

  /** Applies the stored DirichletBoundaryCondition to the given `state`. No-op
   if no DirichletBoundaryCondition is set through
   SetDirichletBoundaryCondition(). */
  void ApplyBoundaryConditions(FemState<Element>* state) const {
    if (dirichlet_bc_ != nullptr) {
      dirichlet_bc_->ApplyBoundaryConditions(state);
    }
  }

 private:
  /* Implements AbstractFemModel::CalcResidual() by casting the AbstractFemState
   to its concrete type. */
  void CalcResidual(const AbstractFemState<T>& state,
                    EigenPtr<VectorX<T>> residual) const final {
    const FemState<Element>& concrete_state = cast_to_concrete_state(state);
    CalcResidual(concrete_state, residual);
  }

  /* Implements AbstractFemModel::CalcTangentMatrix() by casting the
   AbstractFemState to its concrete type. */
  void CalcTangentMatrix(const AbstractFemState<T>& state,
                         Eigen::SparseMatrix<T>* tangent_matrix) const {
    const FemState<Element>& concrete_state = cast_to_concrete_state(state);
    CalcTangentMatrix(concrete_state, tangent_matrix);
  }

  /* Builds the element tangent matrix for the element with index
   `element_index` by combining the stiffness matrix, damping matrix, and the
   mass matrix according to the given `weights`.
   @param[in] state    The FemState to evaluate the residual at.
   @param[in] weights    The ordered weights to combine stiffness matrix,
   damping matrix and mass matrix into the tangent matrix.
   @param[in] element_index    Index of the element whose element tangent
   matrix is being built. */
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

  /* Implements AbstractFemModel::DoUpdateState(). */
  void DoUpdateState(const VectorX<T>& dz,
                     AbstractFemState<T>* state) const final {
    FemState<Element>& mutable_concrete_state =
        cast_to_mutable_concrete_state(state);
    state_updater_->UpdateState(dz, &mutable_concrete_state);
  }

  /* Implements AbstractFemModel::DoAdvanceOneTimeStep(). */
  void DoAdvanceOneTimeStep(const AbstractFemState<T>& prev_state,
                            AbstractFemState<T>* next_state) const final {
    FemState<Element>& next_state_concrete =
        cast_to_mutable_concrete_state(next_state);
    const FemState<Element>& prev_state_concrete =
        cast_to_concrete_state(prev_state);
    state_updater_->AdvanceOneTimeStep(prev_state_concrete,
                                       &next_state_concrete);
  }

  /* Implements AbstractFemModel::ApplyBoundaryConditions() by casting the
   abstract state to the concrete state and invoking the overloaded method for
   the concrete state. */
  void ApplyBoundaryConditions(AbstractFemState<T>* state) const {
    FemState<Element>& concrete_state = cast_to_mutable_concrete_state(state);
    ApplyBoundaryConditions(&concrete_state);
  }

  /* Dynamically cast the given AbstractFemState to the FemState compatible
   with `this` FemModel. Throw an exception if the cast fails. */
  const FemState<Element>& cast_to_concrete_state(
      const AbstractFemState<T>& abstract_state) const {
    const auto* concrete_state_ptr =
        dynamic_cast<const FemState<Element>*>(&abstract_state);
    if (concrete_state_ptr == nullptr) {
      throw std::logic_error(
          "The type of the FemState is incompatible with the type of the "
          "FemModel.");
    }
    return *concrete_state_ptr;
  }

  /* Dynamically cast the given AbstractFemState to the FemState compatible
   with `this` FemModel. Throw an exception if the cast fails. */
  FemState<Element>& cast_to_mutable_concrete_state(
      AbstractFemState<T>* abstract_state) const {
    auto* concrete_state_ptr = dynamic_cast<FemState<Element>*>(abstract_state);
    if (concrete_state_ptr == nullptr) {
      throw std::logic_error(
          "The type of the FemState is incompatible with the type of the "
          "FemModel.");
    }
    return *concrete_state_ptr;
  }

  /* FemElements owned by this model. */
  std::vector<Element> elements_{};
  /* The StateUpdater that updates the states for this model. */
  std::unique_ptr<StateUpdater<FemState<Element>>> state_updater_;
  /* The Dirichlet boundary condition imposed on the model. */
  std::unique_ptr<DirichletBoundaryCondition<FemState<Element>>> dirichlet_bc_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
